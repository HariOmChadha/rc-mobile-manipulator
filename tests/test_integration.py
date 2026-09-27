"""Real TCP sockets, separate Pi process, real JPEG encode/decode and dataset writes."""

import json
import os
import pty
import signal
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
from mobile_robot.phases import PHASES
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
        [sys.executable, "-m", "mobile_robot.server", "--mock", "--config", str(host_path)]
        + (["--read-only"] if getattr(request, "param", None) == "read-only" else []),
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


@pytest.mark.parametrize("answer,quality", [(b"g\n", "good"), (b"b\n", "bad")])
def test_ctrl_c_flushes_and_stops_before_review(network, answer, quality):
    cfg, _, path = network
    cfg_path = path / "laptop.json"
    cfg_path.write_text(json.dumps(cfg))
    root = path / "training_dataset"
    master, slave = pty.openpty()
    log_path = path / "record.log"
    with log_path.open("w") as log:
        proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "mobile_robot.client",
                "--mock",
                "--quality",
                "ask",
                "--config",
                str(cfg_path),
                "--output",
                str(root),
            ],
            stdin=slave,
            stdout=log,
            stderr=log,
        )
        os.close(slave)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                files = list(root.glob("unreviewed/episode_*/telemetry.jsonl"))
                if files and len(files[0].read_text().splitlines()) >= 5:
                    break
                assert proc.poll() is None, log_path.read_text()
                time.sleep(0.03)
            else:
                pytest.fail(log_path.read_text())

            def saved_rows():
                # Only parse newline-terminated rows while the writer is active.
                text = files[0].read_text().rsplit("\n", 1)[0]
                return [json.loads(line) for line in text.splitlines()]

            assert {row["phase"] for row in saved_rows()} == {PHASES[0]}
            for phase_index in (2, 3, 4, 4):
                before = len(saved_rows())
                os.write(master, b"\n")
                deadline = time.monotonic() + 5
                while True:
                    current = saved_rows()
                    if len(current) >= before + 3 and current[-1]["phase_index"] == phase_index:
                        break
                    assert proc.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, log_path.read_text()
                    time.sleep(0.02)
            proc.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 12
            while "Was this run good or bad?" not in log_path.read_text():
                assert proc.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                time.sleep(0.03)
            # Review cannot keep the robot session or the dataset writer active.
            remote = Remote(cfg["control_endpoint"], "integration-test")
            try:
                state = remote.call("status")
                assert state["reason"] == "client_stop"
                assert state["session"] is None
            finally:
                remote.close()
            metadata = json.loads((files[0].parent / "metadata.json").read_text())
            rows = [json.loads(line) for line in files[0].read_text().splitlines()]
            assert metadata["recording_status"] == "complete"
            assert metadata["rows"] == len(rows) >= 5
            assert metadata["phase_names"] == list(PHASES)
            transitions = metadata["phase_transitions"]
            assert [event["phase"] for event in transitions] == list(PHASES)
            for index, event in enumerate(transitions):
                end = transitions[index + 1]["start_row"] if index < 3 else len(rows)
                segment = rows[event["start_row"] : end]
                assert segment and all(row["phase"] == event["phase"] for row in segment)
                assert event["timestamp"] == segment[0]["timestamp"]
                assert event["elapsed_s"] == segment[0]["episode_elapsed_s"]
            os.write(master, answer)
            assert proc.wait(timeout=10) == 0, log_path.read_text()
            episodes = list(root.glob(f"{quality}/episode_*"))
            assert len(episodes) == 1
            assert not list(root.glob("unreviewed/episode_*"))
            assert json.loads((episodes[0] / "metadata.json").read_text())["quality"] == quality
            assert (episodes[0] / "telemetry.jsonl").read_text().splitlines() == [
                json.dumps(row) for row in rows
            ]
            for row in rows:
                for camera in row["images"].values():
                    frame = cv2.imdecode(
                        np.frombuffer((episodes[0] / camera["file"]).read_bytes(), np.uint8), cv2.IMREAD_COLOR
                    )
                    assert frame.shape == (120, 160, 3)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=5)
            os.close(master)


@pytest.mark.parametrize("network", ["read-only"], indirect=True)
def test_read_only_benchmark_over_tcp(network):
    from mobile_robot.benchmark import run as benchmark

    cfg, _, path = network
    report = benchmark(cfg, duration=0.5, output=path / "benchmark", mock=True)
    assert report["passed"]
    assert report["samples"] >= 5
    assert set(report["cameras"]) == {"wrist", "car", "scene"}
    rows = [
        json.loads(line) for line in (Path(report["episode"]) / "telemetry.jsonl").read_text().splitlines()
    ]
    assert all(row["follower_joints"] == dict.fromkeys(JOINTS, 0) for row in rows)
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        assert remote.call("status")["session"] is None
        with pytest.raises(RuntimeError, match="Read-only"):
            remote.start(True)
    finally:
        remote.close()


def test_benchmark_refuses_motion_enabled_server(network):
    from mobile_robot.benchmark import run as benchmark

    cfg, _, path = network
    report = benchmark(cfg, duration=0.2, output=path / "benchmark", mock=True)
    assert not report["passed"]
    assert report["samples"] == 0
    assert "read-only" in report["error"]


@pytest.mark.parametrize("network", ["read-only"], indirect=True)
def test_partial_benchmark_explicitly_marks_omitted_controls(network):
    from mobile_robot.benchmark import run as benchmark

    cfg, _, path = network
    report = benchmark(cfg, duration=0.3, output=path / "partial", mock=True, skip_local_controls=True)
    assert report["passed"]
    assert report["local_controls_tested"] is False
    assert report["rc_last_values"] is None
    rows = [
        json.loads(line) for line in (Path(report["episode"]) / "telemetry.jsonl").read_text().splitlines()
    ]
    assert all(row["leader_joints"] is None for row in rows)


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
