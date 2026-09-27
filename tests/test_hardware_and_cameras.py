import time
from types import SimpleNamespace

import numpy as np
import pytest

from mobile_robot.cameras import Capture, frame_age
from mobile_robot.client import collect_frames
from mobile_robot.config import load_config
from mobile_robot.hardware import Arm
from mobile_robot.protocol import JOINTS


@pytest.fixture
def fake_lerobot(monkeypatch):
    module = pytest.importorskip("lerobot.robots.so_follower")
    instances = []

    class Device:
        calibrated = True
        calibration_available = True
        fail_connect = False

        def __init__(self, cfg):
            self.config = cfg
            self.calibration = {"data": True} if self.calibration_available else {}
            self.calibration_fpath = "/example/follower.json"
            self.bus = SimpleNamespace(is_connected=False)
            self.is_calibrated = self.calibrated
            self.disconnected = False
            self.sent = None
            self.calibrate_argument = None
            instances.append(self)

        def connect(self, calibrate):
            self.calibrate_argument = calibrate
            self.bus.is_connected = True
            if self.fail_connect:
                raise OSError("USB error")

        def get_observation(self):
            return dict.fromkeys(JOINTS, np.float64(12))

        def send_action(self, action):
            self.sent = action
            return action

        def disconnect(self):
            self.disconnected = True
            self.bus.is_connected = False

    monkeypatch.setattr(module, "SO101Follower", Device)
    return Device, instances


def test_real_lerobot_config_adapter_units_hold_and_feedback(fake_lerobot, tmp_path):
    _, instances = fake_lerobot
    arm = Arm({"port": "/fake", "id": "test", "calibration_dir": str(tmp_path)}, max_step=3)
    device = instances[-1]
    try:
        assert device.config.use_degrees is False
        assert device.config.max_relative_target == 3
        assert device.config.disable_torque_on_disconnect is False
        assert device.calibrate_argument is False
        assert arm.hold() == dict.fromkeys(JOINTS, 12)
        assert device.sent == dict.fromkeys(JOINTS, 12)
        assert all(type(v) is float for v in arm.observe().values())
    finally:
        arm.close()
    assert device.disconnected


def test_missing_calibration_fails_before_usb_connect(fake_lerobot, tmp_path):
    cls, instances = fake_lerobot
    cls.calibration_available = False
    with pytest.raises(RuntimeError, match="Missing calibration"):
        Arm({"port": "/fake", "id": "test", "calibration_dir": str(tmp_path)})
    assert instances[-1].calibrate_argument is None


@pytest.mark.parametrize("calibrated", [True, False])
def test_read_only_adapter_never_configures_or_writes_torque(fake_lerobot, tmp_path, monkeypatch, calibrated):
    cls, instances = fake_lerobot
    original = cls.__init__
    calls = []

    def init(self, cfg):
        original(self, cfg)

        def connect():
            self.bus.is_connected = True
            calls.append("bus-connect")

        def disconnect(*, disable_torque):
            assert disable_torque is False
            calls.append("bus-close-without-torque")
            self.bus.is_connected = False

        self.bus.connect = connect
        self.bus.disconnect = disconnect
        self.bus.sync_read = lambda register: {key.removesuffix(".pos"): 12 for key in JOINTS}
        self.is_calibrated = calibrated

    monkeypatch.setattr(cls, "__init__", init)
    monkeypatch.setattr(cls, "connect", lambda *args, **kwargs: pytest.fail("Must not configure motors"))
    monkeypatch.setattr(
        cls, "disconnect", lambda *args: pytest.fail("Must not use torque-changing disconnect")
    )
    if not calibrated:
        with pytest.raises(RuntimeError, match="calibration differs"):
            Arm({"port": "/fake", "id": "test", "calibration_dir": str(tmp_path)}, read_only=True)
    else:
        arm = Arm({"port": "/fake", "id": "test", "calibration_dir": str(tmp_path)}, read_only=True)
        assert arm.observe() == dict.fromkeys(JOINTS, 12)
        assert arm.hold() == dict.fromkeys(JOINTS, 12)
        with pytest.raises(RuntimeError, match="Read-only"):
            arm.send(dict.fromkeys(JOINTS, 0))
        arm.close()
    assert calls == ["bus-connect", "bus-close-without-torque"]
    assert instances[-1].sent is None


@pytest.mark.parametrize("failure", ["fail_connect", "calibrated"])
def test_usb_cleanup_after_connect_failure(fake_lerobot, tmp_path, failure):
    cls, instances = fake_lerobot
    setattr(cls, failure, failure == "fail_connect")
    with pytest.raises((OSError, RuntimeError)):
        Arm({"port": "/fake", "id": "test", "calibration_dir": str(tmp_path)})
    assert instances[-1].disconnected


def test_camera_failure_releases_capture_and_reports_error(monkeypatch):
    import cv2

    class BrokenCamera:
        released = False

        def isOpened(self):
            return True

        def set(self, *args):
            return True

        def get(self, *args):
            return 0

        def read(self):
            return False, None

        def release(self):
            self.released = True

    camera = BrokenCamera()
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: camera)
    capture = Capture(load_config("config/pi.json")["cameras"][0]).start()
    capture.thread.join(2)
    capture.close()
    assert camera.released
    assert "read failed" in capture.error
    assert capture.snapshot() is None


def test_remote_frame_age_does_not_compare_different_host_clocks():
    assert frame_age(
        {"capture_monotonic": 5}, now=1000.2, server_monotonic=5.1, state_received_monotonic=1000
    ) == pytest.approx(0.3)


def test_stale_remote_camera_rejected_even_when_just_received():
    cfg = load_config("config/laptop.json")
    cfg["remote_cameras"] = ["wrist"]
    receiver = SimpleNamespace(
        snapshot=lambda: {
            "wrist": ({"mock": True, "capture_monotonic": 4, "received_monotonic": time.monotonic()}, b"old")
        }
    )
    with pytest.raises(RuntimeError, match="stale"):
        collect_frames(
            cfg, receiver, [], {"server_monotonic": 5, "round_trip_s": 0.01}, time.monotonic(), True
        )


@pytest.mark.parametrize(
    "key,value",
    [
        ("fps", 0),
        ("fps", True),
        ("request_timeout_s", 1),
        ("max_relative_target", -1),
        ("max_frame_age_s", float("nan")),
    ],
)
def test_reject_bad_config(tmp_path, key, value):
    import json

    cfg = {key: value}
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):
        load_config(path)
