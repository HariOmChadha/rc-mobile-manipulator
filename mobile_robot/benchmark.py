"""Read-only end-to-end telemetry and camera benchmark; never starts a control session."""

import argparse
import json
import math
import statistics
import time
import uuid
from pathlib import Path

from .cameras import Capture, Subscriber
from .client import collect_frames
from .config import load_config
from .control import Remote
from .hardware import Arm, MockLeader, RCReader
from .recording import Recorder
from .server import get_token


def percentiles(values):
    if not values:
        return {}
    ordered = sorted(values)
    return {
        "median": statistics.median(values),
        "p95": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
        "max": max(values),
    }


def run(cfg, *, duration=30, output="data/benchmarks", mock=False):
    folder = Path(output) / f"run_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
    folder.mkdir(parents=True)
    remote = Remote(cfg["control_endpoint"], get_token(mock), cfg["request_timeout_s"])
    subscriber = Subscriber(cfg["video_endpoint"]).start()
    local = []
    leader = rc = recorder = None
    rows = []
    failure = None
    started = None
    try:
        state = remote.call("status")
        if not state.get("read_only") or state["mock"] is not mock:
            raise RuntimeError("Start the Pi with ./robot pi --read-only (add --mock only for simulation)")
        leader = MockLeader() if mock else Arm(cfg["leader"], leader=True, read_only=True)
        rc = RCReader(cfg.get("esp32_port"), mock=mock, reset_on_open=False)
        for camera in cfg["cameras"]:
            local.append(Capture(camera, mock=mock).start())
        deadline = time.monotonic() + cfg.get("startup_timeout_s", 10)
        while True:
            state = remote.call("status")
            try:
                collect_frames(cfg, subscriber, local, state, time.monotonic(), mock)
                rc_state = rc.read()
                if cfg.get("esp32_port") and (
                    rc_state["age_s"] is None or rc_state["age_s"] > cfg["max_rc_age_s"]
                ):
                    raise RuntimeError("Waiting for fresh ESP32 telemetry")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)
        recorder = Recorder(
            folder, {"mock": mock, "read_only": True, "config": cfg, "purpose": "benchmark, no actions sent"}
        )
        started = time.monotonic()
        while time.monotonic() - started < duration:
            loop_start = time.monotonic()
            leader_joints = leader.observe()
            leader_read_s = time.monotonic() - loop_start
            rc_state = rc.read()
            if cfg.get("esp32_port") and (
                rc_state["age_s"] is None or rc_state["age_s"] > cfg["max_rc_age_s"]
            ):
                raise RuntimeError("ESP32 telemetry became stale")
            state = remote.call("status")
            frames = collect_frames(cfg, subscriber, local, state, time.monotonic(), mock)
            row = {
                "timestamp": time.time(),
                "sample_seq": len(rows),
                "leader_joints": leader_joints,
                "follower_joints": state["follower_joints"],
                "round_trip_s": state["round_trip_s"],
                "leader_read_s": leader_read_s,
                "rc_state": rc_state,
            }
            recorder.submit(row, frames)
            rows.append(row)
            time.sleep(max(0, 1 / cfg["fps"] - (time.monotonic() - loop_start)))
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
    finally:
        elapsed = time.monotonic() - started if started is not None else 0
        # No start/action/stop RPC and no torque writes, including during cleanup.
        for resource in [remote, subscriber, *local, rc, leader, recorder]:
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    failure = failure or f"Cleanup failed: {error}"
    cameras = {}
    if recorder is not None:
        import cv2

        try:
            saved_rows = [
                json.loads(line) for line in (recorder.path / "telemetry.jsonl").read_text().splitlines()
            ]
            if len(saved_rows) != len(rows):
                raise RuntimeError("Saved sample count differs from collected count")
            for row in saved_rows:
                for name, meta in row["images"].items():
                    c = cameras.setdefault(
                        name,
                        {
                            "files": {},
                            "ages": [],
                            "capture_fps": meta.get("measured_capture_fps"),
                            "width": meta["width"],
                            "height": meta["height"],
                        },
                    )
                    c["files"][meta["file"]] = (meta["height"], meta["width"], 3)
                    c["ages"].append(meta["age_estimate_s"] * 1000)
                    c["capture_fps"] = meta.get("measured_capture_fps")
            for c in cameras.values():
                size = 0
                for filename, shape in c["files"].items():
                    path = recorder.path / filename
                    frame = cv2.imread(str(path))
                    if frame is None or frame.shape != shape:
                        raise RuntimeError(f"Invalid saved image: {path}")
                    size += path.stat().st_size
                c["validated_jpegs"] = len(c.pop("files"))
                c["unique_saved_fps"] = c["validated_jpegs"] / elapsed if elapsed else 0
                c["saved_jpeg_mbit_s"] = size * 8 / elapsed / 1e6 if elapsed else 0
                c["age_estimate_ms"] = percentiles(c.pop("ages"))
        except Exception as error:
            failure = failure or f"Validation failed: {error}"
    report = {
        "passed": failure is None and bool(rows),
        "error": failure,
        "read_only": True,
        "mock": mock,
        "control_endpoint": cfg["control_endpoint"],
        "requested_duration_s": duration,
        "elapsed_s": elapsed,
        "samples": len(rows),
        "telemetry_hz": len(rows) / elapsed if elapsed else 0,
        "round_trip_ms": percentiles([r["round_trip_s"] * 1000 for r in rows]),
        "leader_read_ms": percentiles([r["leader_read_s"] * 1000 for r in rows]),
        "rc_last_values": rows[-1]["rc_state"]["values"] if rows else None,
        "cameras": cameras,
        "episode": str(recorder.path) if recorder else None,
        "note": "Status reads, not a motion test. Image age is an estimate after capture receipt. Saved JPEG bandwidth excludes overhead and dropped frames.",
    }
    report_path = folder / "report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    return {"report": str(report_path), **report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/laptop.local.json")
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--output", default="data/benchmarks")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 3600:
        parser.error("Duration must be between 1 and 3600 seconds")
    report = run(load_config(args.config), duration=args.duration, output=args.output, mock=args.mock)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
