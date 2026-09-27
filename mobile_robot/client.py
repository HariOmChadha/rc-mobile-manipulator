import argparse
import logging
import signal
import threading
import time

from .cameras import Capture, Subscriber, frame_age
from .config import load_config
from .control import Remote
from .hardware import Arm, MockLeader, RCReader
from .recording import Recorder, update_metadata
from .server import get_token

LOG = logging.getLogger(__name__)


def collect_frames(cfg, subscriber, local, state, received_mono, mock):
    frames = subscriber.snapshot()
    selected = {}
    for name in cfg.get("remote_cameras", []):
        if name not in frames:
            raise RuntimeError(f"Waiting for remote camera {name}")
        meta, jpeg = frames[name]
        if meta["mock"] is not mock:
            raise RuntimeError(f"Camera {name} simulation mode differs")
        age = (
            frame_age(
                meta, server_monotonic=state["server_monotonic"], state_received_monotonic=received_mono
            )
            + state["round_trip_s"]
        )
        if age > cfg["max_frame_age_s"]:
            raise RuntimeError(f"Remote camera {name} is stale ({age:.2f}s)")
        selected[name] = ({**meta, "age_estimate_s": age, "clock": "pi"}, jpeg)
    for capture in local:
        item = capture.snapshot()
        if item is None:
            raise RuntimeError(capture.error or f"Waiting for local camera {capture.config['name']}")
        meta, jpeg = item
        age = frame_age(meta)
        if age > cfg["max_frame_age_s"]:
            raise RuntimeError(f"Local camera {meta['name']} is stale ({age:.2f}s)")
        selected[meta["name"]] = ({**meta, "age_estimate_s": age, "clock": "laptop"}, jpeg)
    return selected


def run(cfg, *, mock=False, duration=None, output=None, stop_event=None, check=False):
    stop_event = stop_event or threading.Event()
    local_names = {c["name"] for c in cfg["cameras"]}
    if local_names & set(cfg.get("remote_cameras", [])):
        raise ValueError("Local and remote camera names must differ")
    remote = Remote(cfg["control_endpoint"], get_token(mock), cfg["request_timeout_s"])
    subscriber = Subscriber(cfg["video_endpoint"]).start()
    leader = rc = recorder = None
    local = []
    result = None
    failure = None
    cleanup_errors = []
    try:
        state = remote.call("status")
        if state["mock"] is not mock:
            raise RuntimeError("Pi and laptop must use the same simulation mode")
        if 1 / cfg["fps"] >= state["command_timeout_s"]:
            raise ValueError("Control period must be shorter than Pi watchdog timeout")
        for camera in cfg["cameras"]:
            local.append(Capture(camera, mock=mock).start())
        # Check camera readiness before allowing the first action.
        deadline = time.monotonic() + cfg.get("startup_timeout_s", 10)
        while True:
            state = remote.call("status")
            received_mono = time.monotonic()
            try:
                frames = collect_frames(cfg, subscriber, local, state, received_mono, mock)
                break
            except RuntimeError:
                if stop_event.is_set() or time.monotonic() >= deadline:
                    raise
                stop_event.wait(0.05)
        if check:
            return {
                "mock": mock,
                "read_only": state.get("read_only", False),
                "joint_units": state["units"],
                "calibration_matches": state.get("calibration_matches"),
                "follower_joints": state["follower_joints"],
                "cameras": {
                    name: {
                        "width": meta["width"],
                        "height": meta["height"],
                        "age_estimate_s": meta["age_estimate_s"],
                        "camera": meta.get("camera", {}),
                        "requested_mode": meta.get("requested_mode"),
                        "reported_mode": meta.get("reported_mode"),
                        "measured_capture_fps": meta.get("measured_capture_fps"),
                    }
                    for name, (meta, _) in frames.items()
                },
                "round_trip_s": state["round_trip_s"],
            }
        leader = MockLeader() if mock else Arm(cfg["leader"], leader=True)
        rc = RCReader(cfg.get("esp32_port"), mock=mock)
        if cfg.get("esp32_port") and not mock:
            # Opening serial can reset an ESP32. Wait for a real sample, never invent neutral input.
            deadline = time.monotonic() + cfg.get("startup_timeout_s", 10)
            while rc.read()["values"] is None:
                if stop_event.is_set() or time.monotonic() >= deadline:
                    raise RuntimeError("No valid steering/throttle telemetry from ESP32")
                stop_event.wait(0.02)
        recorder = Recorder(
            output or cfg.get("output_dir", "training_dataset"),
            {
                "mock": mock,
                "recording_status": "recording",
                "quality": "unreviewed",
                "joint_units": "normalized_minus100_100_gripper_0_100",
                "rc_units": "raw_dac_0_255",
                "config": cfg,
                "timestamp_note": "Capture receipt times; cameras are not hardware synchronized. Pi and laptop clocks differ.",
            },
        )
        state = remote.start(mock)
        started = time.monotonic()
        LOG.info("Recording to %s", recorder.path)
        while not stop_event.is_set() and (duration is None or time.monotonic() - started < duration):
            loop_start = time.monotonic()
            action = leader.observe()
            rc_state = rc.read()
            if cfg.get("esp32_port") and (
                rc_state["age_s"] is None or rc_state["age_s"] > cfg["max_rc_age_s"]
            ):
                raise RuntimeError("ESP32 telemetry is stale; ending recording")
            state = remote.action(action)
            received_mono = time.monotonic()
            frames = collect_frames(cfg, subscriber, local, state, received_mono, mock)
            recorder.submit(
                {
                    "timestamp": time.time(),
                    "command_seq": state["seq"],
                    "leader_action": action,
                    "applied_action": state["applied_action"],
                    "follower_joints": state["follower_joints"],
                    "follower_timestamp": state["state_timestamp"],
                    "follower_server_monotonic": state["server_monotonic"],
                    "round_trip_s": state["round_trip_s"],
                    "rc_state": rc_state,
                },
                frames,
            )
            stop_event.wait(max(0, 1 / cfg["fps"] - (time.monotonic() - loop_start)))
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        raise
    finally:
        # Stop motion before waiting for image writes or camera-driver cleanup.
        try:
            remote.stop()
        except Exception as error:
            LOG.warning("Stop request failed; Pi watchdog remains responsible: %s", error)
            cleanup_errors.append(str(error))
        for resource in [remote, subscriber, *local, rc, leader, recorder]:
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    cleanup_errors.append(str(error))
        if recorder:
            status = "error" if failure or cleanup_errors else "complete"
            update_metadata(
                recorder.path,
                recording_status=status,
                recording_error=failure or "; ".join(cleanup_errors) or None,
                rows=recorder.saved,
            )
            result = {"path": str(recorder.path), "rows": recorder.saved, "recording_status": status}
            LOG.info("Saved %s rows to %s (%s)", recorder.saved, recorder.path, status)
        if cleanup_errors and failure is None:
            raise RuntimeError("Recording cleanup failed: " + "; ".join(cleanup_errors))
    return result


def main():
    import json
    from pathlib import Path

    from .review import review_result

    parser = argparse.ArgumentParser(description="SO101 remote teleoperation and three-camera recording")
    parser.add_argument("--config", default="config/laptop.json")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument(
        "--enable-motion", action="store_true", help="Explicitly enable physical follower motion"
    )
    parser.add_argument(
        "--check", action="store_true", help="Check Pi feedback and camera feeds; sends no motion commands"
    )
    parser.add_argument("--duration", type=float, help="End this episode after N seconds; default Ctrl+C")
    parser.add_argument("--output")
    parser.add_argument("--label", help="Experiment label saved in the episode configuration")
    parser.add_argument(
        "--quality",
        choices=["ask", "good", "bad", "unreviewed"],
        help="Review after saving (default: ask for real recordings; unreviewed for simulations)",
    )
    args = parser.parse_args()
    if not args.mock and not args.check and not args.enable_motion:
        parser.error("Use --check first; add --enable-motion when the arm is ready")
    if args.duration is not None and (not 0 < args.duration < float("inf")):
        parser.error("--duration must be finite and positive")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    stop = threading.Event()
    stop_signal = [None]

    def request_stop(sig, _frame):
        stop_signal[0] = sig
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        cfg = load_config(args.config)
        if args.label:
            cfg["experiment_label"] = args.label
        root = Path(
            args.output
            or ("data/mock-recordings" if args.mock else cfg.get("output_dir", "training_dataset"))
        )
        review = not args.check and (not args.mock or args.quality is not None)
        result = run(
            cfg,
            mock=args.mock,
            duration=args.duration,
            output=root / "unreviewed" if review else root,
            stop_event=stop,
            check=args.check,
        )
        if review and result:
            # A second Ctrl+C dismisses review; no hardware remains open here.
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            quality = "unreviewed" if stop_signal[0] == signal.SIGTERM else (args.quality or "ask")
            result = review_result(result, root, quality)
        print(json.dumps(result, indent=2))
    except (ValueError, OSError, RuntimeError, ImportError) as error:
        parser.exit(1, f"Laptop error: {error}\n")


if __name__ == "__main__":
    main()
