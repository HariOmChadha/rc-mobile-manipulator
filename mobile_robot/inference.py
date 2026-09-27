"""Model adapter -> fresh robot observations -> SO101 + ESP32 DAC actions."""

import argparse
import importlib
import json
import logging
import math
import queue
import signal
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from .cameras import Capture, Subscriber
from .car import Car
from .client import collect_frames
from .config import load_config
from .control import Remote
from .protocol import JOINTS, validate_action
from .server import get_token

LOG = logging.getLogger(__name__)
NAMES = (*JOINTS, "steer", "throttle")
UNITS = "normalized_minus100_100_gripper_0_100_and_rc_dac_0_255"


def validate_spec(spec, mock=False):
    for key in ("state_names", "action_names"):
        if len(spec[key]) != 8 or set(spec[key]) != set(NAMES):
            raise ValueError(f"{key} must list each of the six joints, steer and throttle exactly once")
    if spec.get("units") != UNITS:
        raise ValueError("Adapter must return denormalized absolute joint targets and raw DAC outputs")
    if not mock and spec.get("mapping_confirmed") is not True:
        raise ValueError(
            "Confirm training state/action order, units and camera mapping in inference config first"
        )
    for key, low, high in (("neutral_steer", 0, 255), ("neutral_throttle", 50, 145)):
        if type(spec.get(key)) is not int or not low <= spec[key] <= high:
            raise ValueError(f"Set verified {key} as an integer in [{low}, {high}]")
    if not spec.get("camera_map") or any(not isinstance(v, str) for v in spec["camera_map"].values()):
        raise ValueError("camera_map must map model image keys to camera names")
    if {"state", "prompt"} & set(spec["camera_map"]):
        raise ValueError("Camera keys cannot overwrite state or prompt")
    for key in ("max_inference_s", "fps"):
        value = spec.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
    horizon = spec.get("execution_horizon")
    if type(horizon) is not int or not 1 <= horizon <= 300:
        raise ValueError("execution_horizon must be an integer in [1,300]")


def decode_actions(result, spec):
    if isinstance(result, dict):
        result = result["actions"]
    values = np.asarray(result)
    if values.dtype.kind not in "fiu" or values.ndim not in (1, 2):
        raise ValueError("Policy must return numeric actions shaped (8,) or (T,8)")
    if values.ndim == 1:
        values = values[None, :]
    if values.shape[1] != 8 or not 1 <= len(values) <= 4096 or not np.isfinite(values).all():
        raise ValueError("Policy actions must be finite and have exactly eight dimensions")
    actions = []
    for row in values:
        named = dict(zip(spec["action_names"], map(float, row), strict=True))
        arm = validate_action({name: named[name] for name in JOINTS})
        if not 0 <= named["steer"] <= 255 or not 50 <= named["throttle"] <= 145:
            raise ValueError("Policy DAC output outside steering [0,255] / throttle [50,145]")
        actions.append((arm, (round(named["steer"]), round(named["throttle"]))))
    return actions[: spec["execution_horizon"]]


def observation(state, frames, dac, spec):
    named = {**state["follower_joints"], "steer": dac[0], "throttle": dac[1]}
    result = {
        "state": np.asarray([named[name] for name in spec["state_names"]], dtype=np.float32),
        "prompt": spec["task"],
    }
    for key, name in spec["camera_map"].items():
        frame = cv2.imdecode(np.frombuffer(frames[name][1], dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"Cannot decode camera {name}")
        result[key] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return result


class Predictor:
    """Only this worker touches the model; only the main thread touches actuators."""

    def __init__(self, policy):
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.work, args=(policy,), daemon=True)
        self.thread.start()

    def work(self, policy):
        while not self.closed.is_set():
            try:
                obs = self.requests.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                result = policy.infer(obs)
            except Exception as error:
                result = error
            self.results.put(result)

    def close(self):
        # A stuck GPU/server call must not delay actuator shutdown.
        self.closed.set()


class MockPolicy:
    def __init__(self, spec):
        self.spec = spec

    def infer(self, obs):
        values = dict(zip(self.spec["state_names"], obs["state"], strict=True))
        values[JOINTS[0]] = min(20, values[JOINTS[0]] + 1)
        values.update(steer=123, throttle=101)
        return [[values[name] for name in self.spec["action_names"]]]


class MockCar:
    def __init__(self, port, neutral):
        self.neutral = neutral
        self.commands = []
        self.stopped = False

    def connect(self):
        pass

    def arm(self):
        pass

    def drive(self, steer, throttle):
        self.commands.append((steer, throttle))

    def stop(self):
        self.stopped = True

    def close(self):
        self.stop()


def run(
    cfg,
    spec,
    policy,
    *,
    mock=False,
    enable_motion=False,
    duration=None,
    stop_event=None,
    car_factory=None,
    output=None,
):
    validate_spec(spec, mock)
    if not mock and not enable_motion:
        raise ValueError("Physical inference requires --enable-motion")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("duration must be finite and positive")
    camera_names = [c["name"] for c in cfg["cameras"]] + cfg.get("remote_cameras", [])
    if len(camera_names) != len(set(camera_names)) or not set(spec["camera_map"].values()) <= set(
        camera_names
    ):
        raise ValueError("Camera mapping is missing a configured camera or camera names overlap")
    if 1 / spec["fps"] >= 0.35:
        raise ValueError("Inference control rate must exceed the ESP32 watchdog rate")
    stop_event = stop_event or threading.Event()
    remote = Remote(cfg["control_endpoint"], get_token(mock), cfg["request_timeout_s"])
    subscriber = car = predictor = log = None
    local = []
    neutral = spec["neutral_steer"], spec["neutral_throttle"]
    dac = neutral
    ticks = predictions = 0
    try:
        if output:
            path = Path(output)
            path.parent.mkdir(parents=True, exist_ok=True)
            log = path.open("x")
        state = remote.call("status")
        if state["mock"] is not mock or state.get("read_only"):
            raise RuntimeError("Pi must be in matching mock/live mode with control enabled")
        if state["units"] != "normalized_minus100_100_gripper_0_100":
            raise RuntimeError("Pi joint units do not match inference")
        if 1 / spec["fps"] >= state["command_timeout_s"]:
            raise ValueError("Inference control period exceeds Pi watchdog")
        subscriber = Subscriber(cfg["video_endpoint"]).start()
        for camera in cfg["cameras"]:
            local.append(Capture(camera, mock=mock).start())
        deadline = time.monotonic() + cfg.get("startup_timeout_s", 10)
        while True:
            state = remote.call("status")
            try:
                collect_frames(cfg, subscriber, local, state, time.monotonic(), mock)
                break
            except RuntimeError:
                if stop_event.is_set() or time.monotonic() >= deadline:
                    raise
                stop_event.wait(0.03)
        car = (car_factory or (MockCar if mock else Car))(cfg.get("esp32_port"), neutral)
        car.connect()
        if stop_event.is_set():
            return {"ticks": 0, "predictions": 0}
        state = remote.start(mock)
        car.arm()
        predictor = Predictor(policy)
        pending = None
        actions = deque()
        started = time.monotonic()
        LOG.info(
            "Inference started%s; Ctrl+C stops arm commands and sends car neutral", " (MOCK)" if mock else ""
        )
        while not stop_event.is_set() and (duration is None or time.monotonic() - started < duration):
            loop_start = time.monotonic()
            state = remote.call("status")
            frames = collect_frames(cfg, subscriber, local, state, time.monotonic(), mock)
            if pending is not None:
                if time.monotonic() - pending > spec["max_inference_s"]:
                    raise TimeoutError("Policy inference timed out")
                try:
                    result = predictor.results.get_nowait()
                except queue.Empty:
                    pass
                else:
                    if isinstance(result, Exception):
                        raise result
                    actions.extend(decode_actions(result, spec))
                    pending = None
                    predictions += 1
            if not actions and pending is None:
                predictor.requests.put_nowait(observation(state, frames, dac, spec))
                pending = time.monotonic()
            mode = "prediction" if actions else "waiting_neutral"
            arm, dac = actions.popleft() if actions else (state["follower_joints"], neutral)
            # Check freshness before either actuator is commanded. Network failure stops the loop.
            remote.action(arm)
            car.drive(*dac)
            ticks += 1
            if log:
                log.write(
                    json.dumps(
                        {
                            "timestamp": time.time(),
                            "mode": mode,
                            "arm": arm,
                            "steer": dac[0],
                            "throttle": dac[1],
                        }
                    )
                    + "\n"
                )
                log.flush()
            stop_event.wait(max(0, 1 / spec["fps"] - (time.monotonic() - loop_start)))
        return {"ticks": ticks, "predictions": predictions}
    finally:
        # Independent shutdown attempts: a broken arm link must not prevent neutral car output.
        for resource, method in (
            (car, "stop"),
            (remote, "stop"),
            (predictor, "close"),
            (car, "close"),
            (remote, "close"),
            (subscriber, "close"),
            *((c, "close") for c in local),
            (log, "close"),
        ):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as error:
                    LOG.error("Inference cleanup %s failed: %s", method, error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/laptop.local.json")
    parser.add_argument("--inference-config", default="config/inference.json")
    parser.add_argument(
        "--adapter", help="Python module:factory; factory(checkpoint=..., config=...) -> policy.infer"
    )
    parser.add_argument("--checkpoint", help="Checkpoint path or identifier passed unchanged to adapter")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--output", help="New JSONL action log (must not already exist)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    try:
        cfg = load_config(args.config)
        spec = json.loads(Path(args.inference_config).read_text())
        if args.mock:
            spec.update(neutral_steer=128, neutral_throttle=100)  # Simulation only, never physical defaults.
        validate_spec(spec, args.mock)
        if not args.mock and not args.enable_motion:
            parser.error("Use --enable-motion for physical inference; --mock requires a mock Pi server")
        if args.mock:
            policy = MockPolicy(spec)
        else:
            adapter = args.adapter or spec.get("adapter")
            checkpoint = args.checkpoint or spec.get("checkpoint")
            if not adapter or not checkpoint:
                parser.error(
                    "Set adapter module:factory and checkpoint in config or flags; π0.7 loader is training-specific"
                )
            module, name = adapter.rsplit(":", 1)
            policy = getattr(importlib.import_module(module), name)(checkpoint=checkpoint, config=spec)
        output = args.output or f"data/inference/run_{time.time_ns()}.jsonl"
        result = run(
            cfg,
            spec,
            policy,
            mock=args.mock,
            enable_motion=args.enable_motion,
            duration=args.duration,
            stop_event=stop_event,
            output=output,
        )
        print(json.dumps({**result, "action_log": output}))
    except Exception as error:
        parser.exit(1, f"Inference error: {error}\n")


if __name__ == "__main__":
    main()
