"""Select the Pi address without replacing camera, arm or recording settings."""

import argparse
import json
import re
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Select the connection for the next laptop run")
    parser.add_argument("mode", nargs="?", choices=["status", "wifi", "ethernet"], default="status")
    parser.add_argument("--host", help="Save a new Pi IP/hostname for the selected connection")
    args = parser.parse_args(argv)
    if args.host and args.mode == "status":
        parser.error("Choose wifi or ethernet when setting --host")
    profiles_path = Path("config/network.local.json")
    laptop_path = Path("config/laptop.local.json")
    try:
        profiles = json.loads(
            (profiles_path if profiles_path.exists() else Path("config/network.json")).read_text()
        )
        laptop = json.loads((laptop_path if laptop_path.exists() else Path("config/laptop.json")).read_text())
        if args.mode != "status":
            host = args.host if args.host is not None else profiles[args.mode]["host"]
            if (
                not isinstance(host, str)
                or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]*", host)
                or host.startswith("PI_")
            ):
                parser.error(f"Set the Pi address with ./robot {args.mode} --host PI_IP_OR_HOSTNAME")
            laptop["control_endpoint"] = f"tcp://{host}:5555"
            laptop["video_endpoint"] = f"tcp://{host}:5556"
            if args.host is not None:
                profiles[args.mode] = {"host": host}
                profiles_path.write_text(json.dumps(profiles, indent=2) + "\n")
            laptop_path.write_text(json.dumps(laptop, indent=2) + "\n")
        selected = next(
            (
                mode
                for mode in ("wifi", "ethernet")
                if laptop.get("control_endpoint") == f"tcp://{profiles[mode]['host']}:5555"
                and laptop.get("video_endpoint") == f"tcp://{profiles[mode]['host']}:5556"
            ),
            "custom",
        )
        print(f"Selected connection: {selected}")
        print(f"Control: {laptop['control_endpoint']}")
        print(f"Video:   {laptop['video_endpoint']}")
        print("Saved for the next run; stop any recording before switching.")
        print("This selects the Pi address; it does not connect Wi-Fi or change the running Pi server.")
        print("On Wi-Fi, connect both devices to the same network. On Ethernet, connect the cable.")
        print(
            "With the Pi server running, verify with ./robot check (or ./robot check --mock for dummy hardware)."
        )
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
