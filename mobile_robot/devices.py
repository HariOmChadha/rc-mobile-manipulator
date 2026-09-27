"""List port/camera identifiers without sending any arm commands."""

import glob
from pathlib import Path


def main():
    from serial.tools import list_ports

    print("Serial ports:")
    for port in list_ports.comports():
        print(f"  {port.device}: {port.description} serial={port.serial_number}")
    for directory in ("/dev/serial/by-id", "/dev/v4l/by-id", "/dev/v4l/by-path"):
        print(f"\n{directory}:")
        for path in sorted(glob.glob(f"{directory}/*")):
            print(f"  {path} -> {Path(path).resolve()}")
    print("\nVideo nodes (some are metadata nodes, not capture devices):")
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        node = Path(path)
        print(f"  /dev/{node.name}: {(node / 'name').read_text().strip()}")


if __name__ == "__main__":
    main()
