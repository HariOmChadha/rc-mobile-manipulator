"""One-command simulation using the real server and client CLIs in child processes."""

import argparse
import importlib.metadata
import json
import os
import socket
import statistics
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .config import load_config
from .control import Remote
from .protocol import JOINTS


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def validate_episode(path):
    import cv2

    episode = Path(path)
    rows = [json.loads(line) for line in (episode / "telemetry.jsonl").read_text().splitlines()]
    if not rows:
        raise RuntimeError("Simulation did not record any rows")
    metadata = json.loads((episode / "metadata.json").read_text())
    if metadata.get("mock") is not True:
        raise RuntimeError("Expected explicitly simulated data")
    images = {}
    resolutions = {}
    profiles = {}
    for seq, row in enumerate(rows):
        if row["command_seq"] != seq or set(row["follower_joints"]) != set(JOINTS):
            raise RuntimeError("Invalid command sequence or follower feedback")
        if row["rc_state"]["values"] != {"steer": 110.0, "throttle": 140.0}:
            raise RuntimeError("Steering/throttle telemetry mismatch")
        if set(row["images"]) != {"wrist", "car", "scene"}:
            raise RuntimeError("Missing camera in recorded row")
        for name, frame in row["images"].items():
            images.setdefault(name, {})[frame["file"]] = (frame["height"], frame["width"], 3)
            resolutions[name] = f"{frame['width']}x{frame['height']}"
            profiles[name] = frame.get("camera", {})
    for files in images.values():
        for filename, shape in files.items():
            frame = cv2.imread(str(episode / filename))
            if frame is None or frame.shape != shape:
                raise RuntimeError(f"Invalid or missing JPEG: {filename}")
    elapsed = rows[-1]["timestamp"] - rows[0]["timestamp"]
    rtts = sorted(row["round_trip_s"] * 1000 for row in rows)
    return {
        "rows": len(rows),
        "recorded_interval_s": elapsed,
        "observed_control_hz": (len(rows) - 1) / elapsed if elapsed > 0 else None,
        "round_trip_median_ms": statistics.median(rtts),
        "round_trip_p95_ms": rtts[min(len(rtts) - 1, int(0.95 * len(rtts)))],
        "validated_jpegs": {name: len(files) for name, files in images.items()},
        "resolutions": resolutions,
        "camera_profiles": profiles,
        "episode": str(episode),
    }


def run_smoke(
    duration, output, pi_config="config/pi.json", laptop_config="config/laptop.json", inference=False
):
    run_dir = Path(output).resolve() / f"run_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
    run_dir.mkdir(parents=True)
    host = load_config(pi_config)
    laptop = load_config(laptop_config)
    control, video = free_port(), free_port()
    while video == control:
        video = free_port()
    host["control_bind"] = laptop["control_endpoint"] = f"tcp://127.0.0.1:{control}"
    host["video_bind"] = laptop["video_endpoint"] = f"tcp://127.0.0.1:{video}"
    for name, config in (("pi", host), ("laptop", laptop)):
        (run_dir / f"{name}.json").write_text(json.dumps(config, indent=2))
    env = {**os.environ, "ROBOT_TOKEN": "simulation-only"}
    host_log = (run_dir / "pi.log").open("w")
    proc = subprocess.Popen(
        [sys.executable, "-m", "mobile_robot.server", "--mock", "--config", str(run_dir / "pi.json")],
        env=env,
        stdout=host_log,
        stderr=host_log,
    )
    try:
        deadline = time.monotonic() + 15
        while True:
            remote = Remote(laptop["control_endpoint"], "simulation-only", timeout=0.1)
            try:
                remote.call("status")
                break
            except TimeoutError:
                if proc.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f"Simulated Pi did not start; inspect {run_dir / 'pi.log'}")
            finally:
                remote.close()
        base = [
            sys.executable,
            "-m",
            "mobile_robot.client",
            "--mock",
            "--config",
            str(run_dir / "laptop.json"),
        ]
        check = subprocess.run([*base, "--check"], env=env, capture_output=True, text=True, timeout=20)
        (run_dir / "check.log").write_text(check.stdout + check.stderr)
        if check.returncode:
            raise RuntimeError(f"Preflight failed; inspect {run_dir / 'check.log'}")
        if inference:
            action_log = run_dir / "actions.jsonl"
            inference_run = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "mobile_robot.inference",
                    "--mock",
                    "--config",
                    str(run_dir / "laptop.json"),
                    "--duration",
                    str(duration),
                    "--output",
                    str(action_log),
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=duration + 30,
            )
            (run_dir / "inference.log").write_text(inference_run.stdout + inference_run.stderr)
            if inference_run.returncode:
                raise RuntimeError(f"Inference failed; inspect {run_dir / 'inference.log'}")
            rows = [json.loads(line) for line in action_log.read_text().splitlines()]
            if not any(
                row["mode"] == "prediction" and row["steer"] == 123 and row["throttle"] == 101 for row in rows
            ):
                raise RuntimeError("Mock policy did not produce arm + DAC commands")
            report = {"passed": True, "mode": "mock_inference", **json.loads(inference_run.stdout)}
            (run_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
            return report
        recording = subprocess.run(
            [*base, "--duration", str(duration), "--output", str(run_dir / "dataset")],
            env=env,
            capture_output=True,
            text=True,
            timeout=duration + 30,
        )
        (run_dir / "laptop.log").write_text(recording.stdout + recording.stderr)
        if recording.returncode:
            raise RuntimeError(f"Recording failed; inspect {run_dir / 'laptop.log'}")
        report = validate_episode(json.loads(recording.stdout)["path"])
        remote = Remote(laptop["control_endpoint"], "simulation-only")
        try:
            if remote.call("status")["reason"] != "client_stop":
                raise RuntimeError("Client failed to end its session cleanly")
            remote.start(True)
            remote.action(dict.fromkeys(JOINTS, 10.0))
        finally:
            # Deliberately disappear without stop: exercise the Pi watchdog.
            remote.close()
        time.sleep(host["command_timeout_s"] + 0.2)
        remote = Remote(laptop["control_endpoint"], "simulation-only")
        try:
            state = remote.call("status")
            if state["reason"] != "command_timeout" or state["session"] is not None:
                raise RuntimeError("Watchdog did not invalidate the abandoned session")
        finally:
            remote.close()
        report.update(
            {
                "passed": True,
                "mode": "simulation",
                "watchdog_passed": True,
                "preflight": json.loads(check.stdout),
                "requested_duration_s": duration,
                "packages": {
                    name: importlib.metadata.version(name)
                    for name in ("pyzmq", "numpy", "opencv-python-headless", "pyserial")
                },
            }
        )
        (run_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({"report": str(run_dir / "report.json"), **report}, indent=2))
        return report
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        host_log.close()


def main():
    parser = argparse.ArgumentParser(description="Run and validate a complete simulated episode")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--output", default="data/validation")
    parser.add_argument("--pi-config", default="config/pi.json")
    parser.add_argument("--laptop-config", default="config/laptop.json")
    parser.add_argument(
        "--infer", action="store_true", help="Test model-to-arm-and-car control using a mock policy"
    )
    args = parser.parse_args()
    if not 1 <= args.duration <= 3600:
        parser.error("--duration must be between 1 and 3600 seconds")
    run_smoke(args.duration, args.output, args.pi_config, args.laptop_config, args.infer)


if __name__ == "__main__":
    main()
