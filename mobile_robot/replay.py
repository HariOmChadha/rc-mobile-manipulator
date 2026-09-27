"""Render synchronized episode cameras or explicitly replay recorded arm/DAC commands."""

import argparse
import bisect
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .car import Car
from .config import load_config
from .control import Remote
from .inference import MockCar
from .protocol import JOINTS, validate_action
from .server import get_token


class Episode:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.meta = json.loads((self.path / "metadata.json").read_text())
        self.rows = [json.loads(line) for line in (self.path / "telemetry.jsonl").read_text().splitlines()]
        if not self.rows:
            raise ValueError("Episode has no samples")
        stamps = [r["timestamp"] for r in self.rows]
        if any(type(t) not in (int, float) or not math.isfinite(t) for t in stamps):
            raise ValueError("Episode timestamps must be finite numbers")
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError("Episode timestamps must increase strictly")
        self.times = [t - stamps[0] for t in stamps]
        self.period = float(np.median(np.diff(stamps))) if len(stamps) > 1 else 1 / 30
        self.duration = self.times[-1] + self.period
        self.cameras = [name for name in ("scene", "wrist", "car") if name in self.rows[0]["images"]]
        if not self.cameras:
            raise ValueError("Episode has no camera frames")
        for row in self.rows:
            if not set(self.cameras) <= set(row["images"]):
                raise ValueError("A sample is missing a camera")
            for name in self.cameras:
                self.image_path(row["images"][name]["file"])

    def image_path(self, filename):
        path = (self.path / filename).resolve()
        if not path.is_relative_to(self.path) or not path.is_file():
            raise ValueError("Missing frame or image path outside episode")
        return path

    def index_at(self, seconds):
        return max(0, min(len(self.rows) - 1, bisect.bisect_right(self.times, seconds) - 1))

    def commands(self):
        if self.meta.get("recording_status") != "complete":
            raise ValueError("Robot replay requires a completed recording")
        if self.meta.get("joint_units") != "normalized_minus100_100_gripper_0_100":
            raise ValueError("Recorded arm units do not match the robot")
        if self.meta.get("rc_units") != "raw_dac_0_255":
            raise ValueError("Replay requires raw DAC steering/throttle telemetry")
        if self.period >= 0.2 or any(b - a >= 0.2 for a, b in zip(self.times, self.times[1:])):
            raise ValueError("Recorded command gaps are too long for the drive watchdog")
        result = []
        for row in self.rows:
            arm = validate_action(row.get("applied_action"))
            rc = row["rc_state"].get("values")
            if not isinstance(rc, dict):
                raise ValueError("Missing recorded steering/throttle")
            values = [rc.get("steer"), rc.get("throttle")]
            if any(type(v) not in (int, float) or not math.isfinite(v) or v != int(v) for v in values):
                raise ValueError("Recorded DAC values must be finite integers")
            steer, throttle = map(int, values)
            if not 0 <= steer <= 255 or not 50 <= throttle <= 145:
                raise ValueError("Recorded DAC command exceeds current firmware limits")
            result.append((arm, (steer, throttle)))
        return result


def resolve_episode(value):
    path = Path(value)
    if path.is_dir():
        return path
    candidates = [Path("training_dataset") / group / value for group in ("good", "bad", "unreviewed")]
    found = [p for p in candidates if p.is_dir()]
    if len(found) != 1:
        raise ValueError("Specify the full episode directory; no unique matching episode was found")
    return found[0]


def render(episode, output, *, fps=30):
    import av

    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError("Video fps must be in [1,60]")
    output = Path(output).resolve()
    if output.exists():
        raise ValueError(f"Output already exists: {output}; select another output path")
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = 480 * len(episode.cameras), 360
    cache = {}
    count = math.ceil(episode.duration * fps)
    container = av.open(str(output), mode="w")
    try:
        stream = container.add_stream("libx264", rate=fps)
        stream.width, stream.height = width, height
        stream.pix_fmt = "yuv420p"
        stream.options = {"crf": "20", "preset": "fast"}
        for frame_index in range(count):
            seconds = frame_index / fps
            index = episode.index_at(seconds)
            row = episode.rows[index]
            canvas = np.full((height, width, 3), 18, np.uint8)
            for slot, name in enumerate(episode.cameras):
                filename = row["images"][name]["file"]
                if cache.get(name, (None, None))[0] != filename:
                    img = cv2.imread(str(episode.image_path(filename)))
                    if img is None:
                        raise ValueError(f"Cannot decode {filename}")
                    ratio = min(480 / img.shape[1], 270 / img.shape[0])
                    resized = cv2.resize(img, (round(img.shape[1] * ratio), round(img.shape[0] * ratio)))
                    cache[name] = (filename, resized)
                img = cache[name][1]
                left = slot * 480 + (480 - img.shape[1]) // 2
                top = 35 + (270 - img.shape[0]) // 2
                canvas[top : top + img.shape[0], left : left + img.shape[1]] = img
                cv2.putText(
                    canvas,
                    name.upper(),
                    (slot * 480 + 14, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (235, 235, 235),
                    1,
                    cv2.LINE_AA,
                )
            rc = row.get("rc_state", {}).get("values") or {}
            caption = f"{seconds:05.2f} / {episode.duration:.2f}s   {row.get('phase', '')}   STEER {rc.get('steer', '?')}   THROTTLE {rc.get('throttle', '?')} DAC"
            cv2.putText(
                canvas, caption, (14, 329), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, cv2.LINE_AA
            )
            cv2.rectangle(canvas, (14, 347), (width - 14, 352), (70, 70, 70), -1)
            cv2.rectangle(
                canvas,
                (14, 347),
                (14 + int((width - 28) * seconds / episode.duration), 352),
                (220, 170, 70),
                -1,
            )
            frame = av.VideoFrame.from_ndarray(canvas, format="bgr24")
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()
    report = {
        "episode": str(episode.path),
        "video": str(output),
        "fps": fps,
        "frames": count,
        "duration_s": count / fps,
        "cameras": episode.cameras,
        "sync": "All camera views use the same recorded telemetry timeline; no claim of hardware synchronization.",
    }
    output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def stream_robot(episode, cfg, spec, *, enable_motion=False, mock=False, stop_event=None, car_factory=None):
    if not mock and not enable_motion:
        raise ValueError("Robot replay requires --stream --enable-motion")
    if episode.meta.get("mock") and not mock:
        raise ValueError("Cannot replay a simulated episode on physical hardware")
    commands = episode.commands()  # Validate the complete recording before opening devices.
    neutral = spec.get("neutral_steer"), spec.get("neutral_throttle")
    if any(type(v) is not int for v in neutral) or not 0 <= neutral[0] <= 255 or not 50 <= neutral[1] <= 145:
        raise ValueError("Set verified neutral_steer and neutral_throttle in inference.local.json first")
    stop_event = stop_event or threading.Event()
    remote = Remote(cfg["control_endpoint"], get_token(mock), cfg["request_timeout_s"])
    car = None
    sent = 0
    try:
        state = remote.call("status")
        if state["mock"] is not mock or state.get("read_only"):
            raise ValueError("Pi must have matching simulation mode and control enabled")
        if state["units"] != episode.meta["joint_units"]:
            raise ValueError("Live robot and episode use different arm units")
        if state.get("session") is not None:
            raise ValueError("Another robot control session is active")
        error = max(abs(state["follower_joints"][name] - commands[0][0][name]) for name in JOINTS)
        if error > 5:
            raise ValueError(
                f"Arm is {error:.1f} normalized units from the first recorded target. Position it near the recorded starting pose before replay; no commands sent."
            )
        car = (car_factory or (MockCar if mock else Car))(cfg.get("esp32_port"), neutral)
        car.connect()
        if stop_event.is_set():
            return {"commands_sent": 0, "mock": mock}
        remote.start(mock)
        car.arm()
        started = time.monotonic()
        for offset, (arm, dac) in zip(episode.times, commands, strict=True):
            if stop_event.wait(max(0, started + offset - time.monotonic())):
                break
            if time.monotonic() - (started + offset) > 0.1:
                raise TimeoutError(
                    "Replay fell more than 100 ms behind; stopping instead of replaying a backlog"
                )
            state = remote.action(arm)
            if any(abs(state["applied_action"][name] - arm[name]) > 1e-5 for name in JOINTS):
                raise RuntimeError(
                    "Pi limited an arm target; stopping because the recorded trajectory cannot be followed"
                )
            car.drive(*dac)  # Already DAC integers. Never apply model normalization during replay.
            sent += 1
        if sent == len(commands):
            stop_event.wait(max(0, started + episode.duration - time.monotonic()))
        return {"commands_sent": sent, "recorded_commands": len(commands), "mock": mock}
    finally:
        failures = []
        for resource, method in ((car, "stop"), (remote, "stop"), (car, "close"), (remote, "close")):
            if resource:
                try:
                    getattr(resource, method)()
                except Exception as error:
                    failures.append(str(error))
        if failures:
            print(
                "Replay stop/cleanup errors; device watchdogs remain active: " + "; ".join(failures),
                file=sys.stderr,
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode")
    parser.add_argument("--output", help="New synchronized MP4 path")
    parser.add_argument("--play", action="store_true", help="Open rendered video in the default browser")
    parser.add_argument("--stream", action="store_true", help="Replay arm and raw DAC commands to the robot")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--config", default="config/laptop.local.json")
    parser.add_argument("--inference-config", default="config/inference.local.json")
    args = parser.parse_args()
    try:
        episode = Episode(resolve_episode(args.episode))
        if args.stream:
            if args.play or args.output:
                parser.error("Render/play video separately before running a robot command replay")
            stop = threading.Event()
            signal.signal(signal.SIGINT, lambda *_: stop.set())
            signal.signal(signal.SIGTERM, lambda *_: stop.set())
            spec = json.loads(Path(args.inference_config).read_text())
            if args.mock:
                spec.update(neutral_steer=128, neutral_throttle=100)
            result = stream_robot(
                episode,
                load_config(args.config),
                spec,
                enable_motion=args.enable_motion,
                mock=args.mock,
                stop_event=stop,
            )
        else:
            if args.enable_motion or args.mock:
                parser.error("--enable-motion/--mock require --stream")
            output = Path(args.output or f"data/replays/{episode.path.name}.mp4").resolve()
            if not output.exists():
                result = render(episode, output)
            elif args.play and output.with_suffix(".json").exists():
                result = json.loads(output.with_suffix(".json").read_text())
                if result["episode"] != str(episode.path):
                    raise ValueError("Existing video belongs to a different episode")
            else:
                raise ValueError("Video already exists; use --play or select a new --output")
            if args.play:
                import webbrowser

                webbrowser.open(output.as_uri())
        print(json.dumps(result, indent=2))
    except Exception as error:
        parser.exit(1, f"Replay error: {error}\n")


if __name__ == "__main__":
    main()
