"""PI-hosted policy adapter matching the PolicyClient example in message.txt."""

import math
import os
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

from .protocol import JOINTS

ARM_STATE = "observation/arm/joints/position"
GRIPPER_STATE = "observation/arm/gripper/position"
DRIVE_STATE = "observation/base/drive"
ACTION_TOPICS = ("action/arm/joints/position", "action/arm/gripper/position", "action/base/drive")
RC_HALF = 127.5


def local_setting(name, path=None):
    """Read one named .env setting without executing it; shell settings take priority."""
    if os.environ.get(name):
        return os.environ[name]
    path = Path(path) if path else Path(__file__).resolve().parents[1] / ".env"
    if path.is_file():
        for raw in path.read_text().splitlines():
            line = raw.strip()
            if line.startswith("export "):
                line = line[7:].strip()
            key, sep, value = line.partition("=")
            if sep and key.strip() == name:
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                    value = value[1:-1]
                if value:
                    return value
    return None


def load_api_key(path=None):
    value = local_setting("PI_API_KEY", path)
    if not value:
        raise ValueError("Set PI_API_KEY in the shell or repository .env; never commit the key")
    return value


def sdk_types():
    try:
        from pi_sdk.inference import ClientConfig, InferenceInput, PolicyClient
    except ImportError as error:
        raise RuntimeError(
            "Run ./robot setup-inference with your PI registry service-account JSON configured. "
            "The public pi-sdk PyPI package is a reserved placeholder, not this SDK. "
            "Required imports: pi_sdk.inference.ClientConfig, InferenceInput, PolicyClient"
        ) from error
    return ClientConfig, InferenceInput, PolicyClient


def connect(checkpoint, config, *, types=None):
    url = urlsplit(checkpoint)
    if url.scheme != "wss" or not url.hostname or url.username or url.password:
        raise ValueError("PI gateway must be a wss:// URL without embedded credentials")
    ClientConfig, InferenceInput, PolicyClient = types or sdk_types()
    key = load_api_key()
    settings = ClientConfig(
        gateway_url=checkpoint,
        api_key=key,
        robot_task_string=config["task"],
        raw_text=config.get("subtask", config["task"]),
        action_keys=list(ACTION_TOPICS),
        execute_chunk_size=config["execution_horizon"],
    )
    client = PolicyClient(settings)
    return client, InferenceInput


def metadata(client):
    meta = client.server_metadata
    return {
        "camera_names": list(meta.camera_names),
        "action_keys": list(meta.action_keys or []),
        "action_horizon": int(meta.action_horizon) if meta.action_horizon is not None else None,
        "action_dim": getattr(meta, "action_dim", None),
        "input_spec": getattr(meta, "input_spec", None),
        "output_spec": getattr(meta, "output_spec", None),
    }


def inspect(*, checkpoint, config):
    """Connect and report the server schema, without cameras, actuators or inference."""
    client, _ = connect(checkpoint, config)
    try:
        report = metadata(client)
        report["configured_camera_map"] = config["camera_map"]
        report["unmapped_model_cameras"] = sorted(set(report["camera_names"]) - set(config["camera_map"]))
        report["extra_configured_cameras"] = sorted(set(config["camera_map"]) - set(report["camera_names"]))
        return report
    finally:
        client.close()


class Policy:
    def __init__(self, *, checkpoint, config, types=None):
        self.config = config
        neutral = config.get("training_rc_neutral")
        if not isinstance(neutral, dict) or set(neutral) != {"steer", "throttle"}:
            raise ValueError("Set training_rc_neutral from the training split/episode metadata")
        if any(
            type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 255 for v in neutral.values()
        ):
            raise ValueError("training_rc_neutral must contain finite DAC values in [0,255]")
        self.training_neutral = np.asarray([neutral["steer"], neutral["throttle"]], dtype=np.float64)
        # Using a different stop neutral silently changes the learned drive coordinate system.
        live_neutral = np.asarray([config["neutral_steer"], config["neutral_throttle"]], dtype=np.float64)
        tolerance = config.get("neutral_tolerance_dac", 15)
        if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or not 0 <= tolerance <= 15:
            raise ValueError("neutral_tolerance_dac must be finite and between 0 and 15")
        if np.max(np.abs(live_neutral - self.training_neutral)) > tolerance:
            raise ValueError("Verified stop neutral differs from training neutral by more than tolerance")
        self.client, self.Input = connect(checkpoint, config, types=types)
        try:
            self.metadata = metadata(self.client)
            if set(self.metadata["camera_names"]) != set(config["camera_map"]):
                raise ValueError(
                    "Set camera_map keys to the exact server camera_names; no black-frame substitutions. "
                    f"Server expects: {self.metadata['camera_names']}"
                )
            # SDK 0.3.2 servers may omit action_keys while declaring each output topic.
            advertised = set(self.metadata["action_keys"]) | set(self.metadata["output_spec"] or {})
            if not set(ACTION_TOPICS) <= advertised:
                raise ValueError(f"Hosted model does not expose all required action topics: {ACTION_TOPICS}")
        except BaseException:
            self.client.close()
            raise

    def infer(self, obs):
        state = np.asarray(obs["state"])
        if state.shape != (8,) or not np.isfinite(state).all():
            raise ValueError("Expected eight finite robot state values")
        named = dict(zip(self.config["state_names"], state, strict=True))
        states = {
            ARM_STATE: np.asarray([named[name] for name in JOINTS[:5]], dtype=np.float32),
            GRIPPER_STATE: np.asarray([named[JOINTS[5]]], dtype=np.float32),
            DRIVE_STATE: (
                (np.asarray([named["steer"], named["throttle"]]) - self.training_neutral) / RC_HALF
            ).astype(np.float32),
        }
        images = {}
        for name in self.metadata["camera_names"]:
            frame = np.asarray(obs[name])
            if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3 or not frame.size:
                raise ValueError(f"Model camera {name} must be a nonempty RGB uint8 image")
            images[name] = frame
        result = self.client.infer(self.Input(images=images, states=states, raw_text=obs["prompt"]))
        arrays = []
        for topic, width in zip(ACTION_TOPICS, (5, 1, 2), strict=True):
            if topic not in result.actions_by_key:
                raise ValueError(f"Hosted response is missing {topic}")
            values = np.asarray(result.actions_by_key[topic])
            if (
                values.dtype.kind not in "fiu"
                or values.ndim != 2
                or values.shape[1] != width
                or not 1 <= len(values) <= 4096
                or not np.isfinite(values).all()
            ):
                raise ValueError(f"Invalid hosted action shape/values for {topic}; expected (T,{width})")
            arrays.append(values.astype(np.float64))
        if len({len(a) for a in arrays}) != 1:
            raise ValueError("Hosted action topics have different chunk lengths")
        # The supplied training example uses centered DAC offsets, not raw DAC outputs.
        dac = self.training_neutral + arrays[2] * RC_HALF
        raw = np.concatenate([arrays[0], arrays[1], dac], axis=1)
        canonical = (*JOINTS, "steer", "throttle")
        # No clipping: the robot runner enforces physical joint and current firmware DAC limits.
        return raw[:, [canonical.index(name) for name in self.config["action_names"]]]

    def close(self):
        self.client.close()


def create(*, checkpoint, config):
    return Policy(checkpoint=checkpoint, config=config)
