"""Real TCP sockets, separate Pi process, real JPEG encode/decode and dataset writes."""

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pytest
import zmq

from mobile_robot.client import run
from mobile_robot.config import load_config
from mobile_robot.control import Remote
from mobile_robot.protocol import JOINTS


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


@pytest.fixture
def network(tmp_path, monkeypatch, request):
    monkeypatch.setenv("ROBOT_TOKEN", "integration-test")
    control_port, video_port = free_port(), free_port()
    while video_port == control_port:
        video_port = free_port()
    host = load_config("config/pi.json")
    host["control_bind"] = f"tcp://127.0.0.1:{control_port}"
    host["video_bind"] = f"tcp://127.0.0.1:{video_port}"
    width, height, fps = (640, 480, 15) if getattr(request, "param", None) == "full" else (160, 120, 30)
    for cam in host["cameras"]:
        cam.update(width=width, height=height, fps=fps)
    cfg = load_config("config/laptop.json")
    cfg.update(control_endpoint=host["control_bind"], video_endpoint=host["video_bind"], startup_timeout_s=2)
    for cam in cfg["cameras"]:
        cam.update(width=width, height=height, fps=fps)
    host_path = tmp_path / "pi.json"
    host_path.write_text(json.dumps(host))
    log = (tmp_path / "server.log").open("w+")
    proc = subprocess.Popen(
        [sys.executable, "-m", "mobile_robot.server", "--mock", "--config", str(host_path)],
        stdout=log,
        stderr=log,
    )
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            remote = Remote(cfg["control_endpoint"], "integration-test", 0.1)
            try:
                remote.call("status")
                break
            except TimeoutError:
                if proc.poll() is not None:
                    log.seek(0)
                    pytest.fail(log.read())
                time.sleep(0.02)
            finally:
                remote.close()
        else:
            pytest.fail("Pi did not start")
        yield cfg, proc, tmp_path
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        log.close()


def test_three_camera_recording_over_tcp(network):
    cfg, _, path = network
    check = run(cfg, mock=True, check=True)
    assert set(check["cameras"]) == {"wrist", "car", "scene"}
    result = run(cfg, mock=True, duration=0.8, output=path / "dataset")
    episode = Path(result["path"])
    rows = [json.loads(line) for line in (episode / "telemetry.jsonl").read_text().splitlines()]
    assert len(rows) >= 10
    assert len(rows) == result["rows"]
    for index, row in enumerate(rows):
        assert row["command_seq"] == index
        assert row["rc_state"]["values"] == {"steer": 110, "throttle": 140}
        assert set(row["follower_joints"]) == set(JOINTS)
        assert row["follower_joints"] == row["applied_action"]
        for name, meta in row["images"].items():
            jpeg = (episode / meta["file"]).read_bytes()
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            assert frame.shape == (120, 160, 3)
            assert meta["clock"] == ("laptop" if name == "scene" else "pi")
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        assert remote.call("status")["reason"] == "client_stop"
    finally:
        remote.close()


def test_bad_json_wrong_token_and_recovery(network):
    cfg, _, _ = network
    context = zmq.Context()
    sock = context.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 1000)
    sock.connect(cfg["control_endpoint"])
    try:
        sock.send(b"not json")
        assert sock.recv_json()["ok"] is False
        sock.send_json({"version": 1, "token": "wrong", "op": "start", "mock": True})
        assert sock.recv_json()["ok"] is False
    finally:
        sock.close(0)
        context.term()
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        assert remote.call("status")["reason"] == "idle"
    finally:
        remote.close()


def test_stalled_video_consumer_does_not_block_arm(network):
    cfg, _, _ = network
    context = zmq.Context()
    stalled = context.socket(zmq.SUB)
    stalled.setsockopt(zmq.SUBSCRIBE, b"")
    stalled.setsockopt(zmq.RCVHWM, 1)
    stalled.connect(cfg["video_endpoint"])
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        remote.start(True)
        for _ in range(35):
            state = remote.action(dict.fromkeys(JOINTS, 20))
            assert state["reason"] == "active"
            time.sleep(0.02)
        remote.stop()
    finally:
        remote.close()
        stalled.close(0)
        context.term()


def test_killed_laptop_triggers_watchdog_and_requires_new_session(network):
    cfg, _, path = network
    cfg_path = path / "laptop.json"
    cfg_path.write_text(json.dumps(cfg))
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "mobile_robot.client",
            "--mock",
            "--duration",
            "30",
            "--config",
            str(cfg_path),
            "--output",
            str(path / "crash"),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            files = list((path / "crash").glob("*/telemetry.jsonl"))
            if files and len(files[0].read_text().splitlines()) >= 3:
                break
            assert proc.poll() is None
            time.sleep(0.03)
        else:
            pytest.fail("Client did not begin recording")
        proc.kill()
        proc.wait(3)
        time.sleep(0.65)
        remote = Remote(cfg["control_endpoint"], "integration-test")
        try:
            state = remote.call("status")
            assert state["reason"] == "command_timeout"
            assert state["session"] is None
            remote.start(True)
            assert remote.action(dict.fromkeys(JOINTS, 0))["reason"] == "active"
            remote.stop()
        finally:
            remote.close()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def test_missing_camera_prevents_motion(network):
    cfg, _, path = network
    cfg.update(remote_cameras=["missing"], startup_timeout_s=0.3)
    with pytest.raises(RuntimeError, match="missing"):
        run(cfg, mock=True, duration=0.1, output=path / "not_created")
    assert not (path / "not_created").exists()
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        assert remote.call("status")["reason"] == "idle"
    finally:
        remote.close()


def test_server_loss_fails_with_bounded_wait(network):
    cfg, proc, _ = network
    remote = Remote(cfg["control_endpoint"], "integration-test", timeout=0.1)
    try:
        remote.start(True)
        proc.kill()
        proc.wait(3)
        start = time.monotonic()
        with pytest.raises(TimeoutError):
            remote.action(dict.fromkeys(JOINTS, 0))
        assert time.monotonic() - start < 1
        assert remote.socket is None
    finally:
        remote.close()


def test_real_client_cannot_control_mock_host(network):
    cfg, _, path = network
    with pytest.raises(RuntimeError, match="simulation mode"):
        run(cfg, mock=False, output=path / "not_created")
    assert not (path / "not_created").exists()


@pytest.mark.parametrize("network", ["full"], indirect=True)
def test_full_resolution_recording(network):
    cfg, _, path = network
    result = run(cfg, mock=True, duration=10, output=path / "full_resolution")
    episode = Path(result["path"])
    rows = [json.loads(line) for line in (episode / "telemetry.jsonl").read_text().splitlines()]
    assert len(rows) >= 100  # Generous lower bound; not a Pi throughput claim.
    for name in ("wrist", "car", "scene"):
        assert len({row["images"][name]["seq"] for row in rows}) >= 50
        for row in (rows[0], rows[-1]):
            frame = cv2.imread(str(episode / row["images"][name]["file"]))
            assert frame.shape == (480, 640, 3)
    print(f"Full-resolution simulation: {len(rows)} rows in 10 seconds")
