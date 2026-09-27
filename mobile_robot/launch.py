"""Convenience commands; credentials stay out of command lines and tracked files."""

import argparse
import json
import os
import re
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Prepared robot commands (run from the repository root)")
    parser.add_argument(
        "command",
        choices=[
            "simulate",
            "check",
            "record",
            "pi",
            "devices",
            "test",
            "set-pi",
            "cameras",
            "network",
            "wifi",
            "ethernet",
            "benchmark",
            "classify",
        ],
    )
    args, extra = parser.parse_known_args()
    token_file = Path(".robot-token")
    if not os.environ.get("ROBOT_TOKEN") and token_file.exists():
        os.environ["ROBOT_TOKEN"] = token_file.read_text().strip()
    if args.command in ("network", "wifi", "ethernet"):
        from .network import main as network_main

        network_main(extra if args.command == "network" else [args.command, *extra])
        return
    if args.command == "set-pi":
        if len(extra) != 1 or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]*", extra[0]):
            parser.error("Usage: ./robot set-pi PI_IP_OR_HOSTNAME")
        path = Path("config/laptop.local.json")
        cfg = json.loads(path.read_text() if path.exists() else Path("config/laptop.json").read_text())
        cfg["control_endpoint"] = f"tcp://{extra[0]}:5555"
        cfg["video_endpoint"] = f"tcp://{extra[0]}:5556"
        path.write_text(json.dumps(cfg, indent=2) + "\n")
        print(f"Laptop configured for Pi {extra[0]}. Confirm USB/camera paths before ./robot check.")
        return
    modules = {
        "simulate": "mobile_robot.smoke",
        "check": "mobile_robot.client",
        "record": "mobile_robot.client",
        "pi": "mobile_robot.server",
        "devices": "mobile_robot.devices",
        "test": "pytest",
        "cameras": "mobile_robot.camera_config",
        "benchmark": "mobile_robot.benchmark",
        "classify": "mobile_robot.review",
    }
    defaults = []
    if args.command in ("check", "record", "benchmark"):
        defaults = ["--config", "config/laptop.local.json"]
        # Respect experiment configs supplied by the user when checking the address.
        supplied = argparse.ArgumentParser(add_help=False)
        supplied.add_argument("--config", default=defaults[-1])
        chosen, _ = supplied.parse_known_args(extra)
        cfg = json.loads(Path(chosen.config).read_text())
        if "PI_ADDRESS" in cfg["control_endpoint"]:
            parser.error("Pi address is not set. Run ./robot set-pi PI_IP_OR_HOSTNAME when it is available.")
    if args.command == "check":
        defaults.append("--check")
    if args.command == "pi":
        defaults = ["--config", "config/pi.local.json"]
    os.execv(sys.executable, [sys.executable, "-m", modules[args.command], *defaults, *extra])


if __name__ == "__main__":
    main()
