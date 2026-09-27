"""Compare local camera capture settings without opening an arm or ESP32."""

import json
import time
import uuid
from pathlib import Path

from .cameras import Capture
from .config import load_config


def probe(config, duration, output, *, mock=False, label=None):
    cfg = load_config(config)
    if not cfg["cameras"]:
        raise ValueError("No cameras configured")
    folder = Path(output) / f"{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
    captures = []
    try:
        for camera in cfg["cameras"]:
            captures.append(Capture(camera, mock=mock).start())
        deadline = time.monotonic() + 10
        while any(c.snapshot() is None for c in captures):
            errors = [c.error for c in captures if c.error]
            if errors:
                raise RuntimeError("; ".join(errors))
            if time.monotonic() >= deadline:
                raise TimeoutError("Cameras did not produce frames within 10 seconds")
            time.sleep(0.01)
        first = {c.config["name"]: c.snapshot()[0] for c in captures}
        observed = {name: {} for name in first}
        started = time.monotonic()
        while time.monotonic() - started < duration:
            for capture in captures:
                if capture.error:
                    raise RuntimeError(capture.error)
                meta, jpeg = capture.snapshot()
                if time.monotonic() - meta["capture_monotonic"] > cfg["max_frame_age_s"]:
                    raise RuntimeError(f"Camera {meta['name']} is stale")
                observed[meta["name"]][meta["seq"]] = len(jpeg)
            time.sleep(0.005)
        folder.mkdir(parents=True, exist_ok=False)
        report = {
            "mock": mock,
            "label": label,
            "duration_s": duration,
            "cameras": {},
            "note": "Local capture/encoding only; JPEG payload estimate excludes network overhead. FOV is not measured.",
        }
        for capture in captures:
            meta, jpeg = capture.snapshot()
            name = meta["name"]
            elapsed = meta["capture_monotonic"] - first[name]["capture_monotonic"]
            count = meta["seq"] - first[name]["seq"]
            if count <= 0 or elapsed <= 0:
                raise RuntimeError(f"Camera {name} did not advance during the test")
            sample = f"{name}.jpg"
            (folder / sample).write_bytes(jpeg)
            mean_bytes = sum(observed[name].values()) / len(observed[name])
            report["cameras"][name] = {
                **meta,
                "sample_file": sample,
                "observed_capture_fps": count / elapsed,
                "mean_jpeg_bytes": mean_bytes,
                "estimated_jpeg_mbit_s": mean_bytes * count / elapsed * 8 / 1_000_000,
            }
        (folder / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        return folder / "report.json"
    finally:
        for capture in captures:
            capture.close()
