"""Explicit interactive calibration, separate from unattended startup."""

import argparse

from .config import load_config
from .hardware import Arm


def main():
    parser = argparse.ArgumentParser(description="Calibrate/reuse saved SO101 calibration with LeRobot")
    parser.add_argument("--config", required=True)
    parser.add_argument("--role", choices=["leader", "follower"], required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    arm = Arm(
        cfg[args.role], leader=args.role == "leader", calibrate=True, max_step=cfg["max_relative_target"]
    )
    try:
        print(f"Calibration ready: {arm.device.calibration_fpath}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
