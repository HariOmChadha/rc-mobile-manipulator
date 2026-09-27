"""Hardware imports are lazy so simulation does not need LeRobot or an arm."""

import math
import time
from pathlib import Path

from .protocol import JOINTS


class MockArm:
    def __init__(self, max_step=5):
        self.position = dict.fromkeys(JOINTS, 0.0)
        self.max_step = max_step
        self.actions = []
        self.holds = 0

    def observe(self):
        return dict(self.position)

    def send(self, target):
        applied = {
            k: self.position[k] + max(-self.max_step, min(self.max_step, v - self.position[k]))
            for k, v in target.items()
        }
        self.position.update(applied)
        self.actions.append(dict(applied))
        return applied

    def hold(self):
        self.holds += 1
        return self.observe()

    def close(self):
        pass


class Arm:
    def __init__(self, config, *, leader=False, calibrate=False, max_step=5, read_only=False):
        self.device = None
        self.read_only = read_only
        self.leader = leader
        if read_only and calibrate:
            raise ValueError("Read-only mode cannot calibrate motors")
        if leader:
            from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

            cls, cfg_cls = SO101Leader, SO101LeaderConfig
            kwargs = {}
        else:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

            cls, cfg_cls = SO101Follower, SO101FollowerConfig
            # Retain holding torque on normal shutdown; do not let a loaded arm drop.
            kwargs = {"max_relative_target": max_step, "disable_torque_on_disconnect": False}
        kwargs.update(port=config["port"], id=config["id"], use_degrees=False)
        if config.get("calibration_dir"):
            kwargs["calibration_dir"] = Path(config["calibration_dir"])
        self.device = cls(cfg_cls(**kwargs))
        try:
            if not calibrate and not self.device.calibration:
                raise RuntimeError(
                    f"Missing calibration: {self.device.calibration_fpath}. Run the calibrate command first."
                )
            if read_only:
                # Bus handshake only pings/reads registers. Device.connect() also
                # configures motors and toggles torque, so never call it here.
                self.device.bus.connect()
            else:
                self.device.connect(calibrate=calibrate)
            if not self.device.is_calibrated:
                raise RuntimeError(
                    "Motor calibration differs from the saved file. Run calibrate before teleoperation."
                )
        except BaseException:
            self.close()
            raise

    def observe(self):
        if self.read_only:
            return {f"{k}.pos": float(v) for k, v in self.device.bus.sync_read("Present_Position").items()}
        data = self.device.get_action() if self.leader else self.device.get_observation()
        return {key: float(data[key]) for key in JOINTS}

    def send(self, target):
        if self.read_only:
            raise RuntimeError("Read-only arm cannot send movement commands")
        return {k: float(v) for k, v in self.device.send_action(target).items()}

    def hold(self):
        position = self.observe()
        if not self.read_only:
            self.send(position)
        return position

    def close(self):
        if self.device is not None and self.device.bus.is_connected:
            if self.read_only:
                self.device.bus.disconnect(disable_torque=False)
            else:
                self.device.disconnect()


class MockLeader:
    def observe(self):
        t = time.monotonic()
        return {key: (30 + 10 * math.sin(t)) if key == "gripper.pos" else 10 * math.sin(t) for key in JOINTS}

    def close(self):
        pass


class RCReader:
    """Nonblocking, bounded serial reads; preserves partial lines between polls."""

    def __init__(self, port=None, *, mock=False, reset_on_open=True):
        self.mock = mock
        self.serial = None
        self.buffer = b""
        self.latest = None
        self.updated = None
        if port and not mock:
            import serial

            if reset_on_open:
                self.serial = serial.Serial(port, 115200, timeout=0)
            else:
                self.serial = serial.Serial(None, 115200, timeout=0)
                self.serial.dtr = self.serial.rts = False
                self.serial.port = port
                self.serial.open()

    def read(self):
        from .protocol import parse_rc_line

        now = time.monotonic()
        if self.mock:
            self.latest, self.updated = {"steer": 110.0, "throttle": 140.0}, now
        elif self.serial:
            self.buffer += self.serial.read(min(self.serial.in_waiting, 8192))
            lines = self.buffer.split(b"\n")
            self.buffer = lines.pop()
            if len(self.buffer) > 8192:
                self.buffer = b""
            for line in lines:
                try:
                    self.latest = parse_rc_line(line)
                    self.updated = now
                except (ValueError, UnicodeError):
                    continue
        return {
            "values": self.latest,
            "age_s": None if self.updated is None else now - self.updated,
            "units": "raw_dac_0_255",
        }

    def close(self):
        if self.serial:
            self.serial.close()
