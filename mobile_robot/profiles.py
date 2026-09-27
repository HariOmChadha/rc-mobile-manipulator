"""Declarative camera presets; explicit per-camera settings override profile defaults."""

import json
from copy import deepcopy
from pathlib import Path


def catalog():
    path = Path(__file__).resolve().parents[1] / "config" / "camera_profiles.json"
    return json.loads(path.read_text())


def resolve(camera):
    profile = camera.get("profile")
    if profile is None:
        return deepcopy(camera)
    profiles = catalog()
    if profile not in profiles:
        raise ValueError(f"Unknown camera profile {profile!r}. Run ./robot cameras list")
    return {**deepcopy(profiles[profile]), **deepcopy(camera)}
