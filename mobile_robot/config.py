import json
import math
import re
from pathlib import Path

from .profiles import resolve


def load_config(path):
    with open(path) as stream:
        cfg = json.load(stream)
    cfg.setdefault("fps", 30)
    cfg.setdefault("command_timeout_s", 0.5)
    cfg.setdefault("request_timeout_s", 0.3)
    cfg.setdefault("max_relative_target", 5.0)
    cfg.setdefault("max_frame_age_s", 0.5)
    cfg.setdefault("max_rc_age_s", 0.5)
    cfg.setdefault("cameras", [])
    cfg["cameras"] = [resolve(camera) for camera in cfg["cameras"]]
    for key in (
        "fps",
        "command_timeout_s",
        "request_timeout_s",
        "max_relative_target",
        "max_frame_age_s",
        "max_rc_age_s",
    ):
        value = cfg[key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{key} must be finite and positive")
    if cfg["request_timeout_s"] >= cfg["command_timeout_s"]:
        raise ValueError("request_timeout_s must be shorter than command_timeout_s")
    names = []
    for camera in cfg["cameras"]:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", camera["name"]):
            raise ValueError("Camera names may only contain letters, numbers, underscore and dash")
        names.append(camera["name"])
        for key, default in (("width", 640), ("height", 480), ("fps", 15), ("jpeg_quality", 80)):
            camera.setdefault(key, default)
            if isinstance(camera[key], bool) or not isinstance(camera[key], int) or camera[key] <= 0:
                raise ValueError(f"Camera {key} must be a positive integer")
        if camera["jpeg_quality"] > 100:
            raise ValueError("JPEG quality must be <= 100")
        camera.setdefault("fourcc", "MJPG")
        if len(camera["fourcc"]) != 4:
            raise ValueError("fourcc must contain four characters")
        camera.setdefault("strict_mode", False)
        if type(camera["strict_mode"]) is not bool:
            raise ValueError("Camera strict_mode must be true or false")
        device = camera.get("device")
        if not ((type(device) is int and device >= 0) or (isinstance(device, str) and device)):
            raise ValueError("Camera device must be a nonnegative index or a device path")
    if len(names) != len(set(names)):
        raise ValueError("Camera names must be unique")
    for key in ("leader", "follower"):
        if key in cfg and cfg[key].get("calibration_dir"):
            directory = Path(cfg[key]["calibration_dir"]).expanduser()
            if not directory.is_absolute():
                directory = Path(path).resolve().parent / directory
            cfg[key]["calibration_dir"] = str(directory)
    return cfg
