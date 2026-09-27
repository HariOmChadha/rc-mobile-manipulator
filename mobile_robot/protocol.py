"""Small, versioned messages. Joint values use LeRobot normalized units."""

import math

VERSION = 1
JOINTS = tuple(
    f"{name}.pos"
    for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
)


def validate_action(action):
    if not isinstance(action, dict) or set(action) != set(JOINTS):
        raise ValueError("Expected all six named SO101 joint targets")
    result = {}
    for name, value in action.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"Invalid joint value: {name}")
        low = 0 if name == "gripper.pos" else -100
        if not low <= value <= 100:
            raise ValueError(f"{name} outside normalized range [{low}, 100]")
        result[name] = float(value)
    return result


def parse_rc_line(line):
    """Firmware emits STEER,THROTTLE, both raw DAC values (0..255)."""
    if isinstance(line, bytes):
        line = line.decode("ascii")
    parts = line.strip().split(",")
    if len(parts) != 2:
        raise ValueError("Expected steer,throttle")
    steer, throttle = (float(part) for part in parts)
    if any(not math.isfinite(v) or not 0 <= v <= 255 for v in (steer, throttle)):
        raise ValueError("RC values must be finite DAC values in [0,255]")
    return {"steer": steer, "throttle": throttle}
