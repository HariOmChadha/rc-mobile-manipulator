import json
from pathlib import Path

import cv2
import pytest

from mobile_robot.camera_config import set_camera
from mobile_robot.camera_probe import probe
from mobile_robot.cameras import describe_mode
from mobile_robot.config import load_config
from mobile_robot.profiles import resolve


def test_profile_override_does_not_mutate_catalog():
    camera = {"name": "wrist", "device": 0, "profile": "innomaker-u20cam-720p", "jpeg_quality": 70}
    result = resolve(camera)
    assert (result["width"], result["height"], result["fps"]) == (1280, 720, 30)
    assert result["jpeg_quality"] == 70
    result["fov_claim"]["horizontal_degrees"] = 1
    assert resolve(camera)["fov_claim"]["horizontal_degrees"] == 102


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="Unknown camera profile"):
        resolve({"profile": "made-up"})


def test_change_one_camera_preserves_ports_and_rebases_calibration(tmp_path):
    source = tmp_path / "pi.json"
    original = json.loads(Path("config/pi.json").read_text())
    source.write_text(json.dumps(original))
    dest = tmp_path / "experiments" / "wide.json"
    set_camera(source, dest, "wrist", "innomaker-u20cam-720p")
    cfg = load_config(dest)
    assert cfg["cameras"][0]["width"] == 1280
    assert cfg["cameras"][0]["device"] == original["cameras"][0]["device"]
    assert json.loads(dest.read_text())["cameras"][1] == original["cameras"][1]
    assert source.read_text() == json.dumps(original)
    assert (
        Path(cfg["follower"]["calibration_dir"]).resolve()
        == Path(load_config(source)["follower"]["calibration_dir"]).resolve()
    )


def test_bad_update_never_replaces_active_config(tmp_path):
    source = tmp_path / "pi.json"
    source.write_bytes(Path("config/pi.json").read_bytes())
    original = source.read_bytes()
    with pytest.raises(ValueError):
        set_camera(source, source, "wrist", fps=-1)
    assert source.read_bytes() == original
    assert not source.with_name(source.name + ".tmp").exists()
    with pytest.raises(ValueError, match="exactly one"):
        set_camera(source, source, "missing", "generic-vga")


def test_switching_back_does_not_retain_wide_camera_identity(tmp_path):
    source = tmp_path / "pi.json"
    source.write_bytes(Path("config/pi.json").read_bytes())
    set_camera(source, source, "wrist", "innomaker-u20cam-720p")
    set_camera(source, source, "wrist", "generic-vga")
    camera = load_config(source)["cameras"][0]
    assert camera["width"] == 640
    assert "fov_claim" not in camera
    assert camera["model"] == "Unspecified UVC camera"


def test_strict_negotiation_failure_is_visible(caplog):
    class Driver:
        def get(self, prop):
            return {
                cv2.CAP_PROP_FRAME_WIDTH: 640,
                cv2.CAP_PROP_FRAME_HEIGHT: 480,
                cv2.CAP_PROP_FPS: 15,
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*"MJPG"),
            }[prop]

    camera = resolve({"name": "wrist", "device": 0, "profile": "innomaker-u20cam-720p"})
    with pytest.raises(RuntimeError, match="mode mismatch"):
        describe_mode(Driver(), camera)
    camera["strict_mode"] = False
    actual = describe_mode(Driver(), camera)
    assert actual["width"] == 640
    assert "mode mismatch" in caplog.text


def test_unreported_driver_values_are_not_claimed_as_verified(caplog):
    class Driver:
        def get(self, prop):
            return 0

    cfg = resolve({"name": "wrist", "profile": "innomaker-u20cam-720p"})
    assert all(value is None for value in describe_mode(Driver(), cfg).values())
    assert "does not report" in caplog.text


def test_camera_only_probe_supports_mixed_modes_and_records_identity(tmp_path):
    report_path = probe("config/experiments/pi-wrist-wide.json", 1, tmp_path, mock=True, label="comparison")
    report = json.loads(report_path.read_text())
    assert report["mock"] is True
    assert report["label"] == "comparison"
    wrist, car = report["cameras"]["wrist"], report["cameras"]["car"]
    assert wrist["camera"]["profile"] == "innomaker-u20cam-720p"
    assert wrist["requested_mode"]["width"] == wrist["reported_mode"]["width"] == 1280
    assert wrist["observed_capture_fps"] > 10
    assert cv2.imread(str(report_path.parent / wrist["sample_file"])).shape == (720, 1280, 3)
    assert cv2.imread(str(report_path.parent / car["sample_file"])).shape == (480, 640, 3)
