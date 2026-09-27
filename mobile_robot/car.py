"""Acknowledged ESP32 override, with an independent 350 ms firmware watchdog."""

import argparse
import secrets
import time

from .protocol import parse_rc_line


class Car:
    def __init__(self, port, neutral, *, serial_device=None):
        import serial

        if (
            len(neutral) != 2
            or any(type(v) is not int for v in neutral)
            or not (0 <= neutral[0] <= 255 and 50 <= neutral[1] <= 145)
        ):
            raise ValueError("Verified integer neutral DAC values are required")
        if serial_device is None:
            self.serial = serial.Serial(None, 115200, timeout=0, write_timeout=0.1, exclusive=True)
            self.serial.dtr = self.serial.rts = False
            self.serial.port = port
            self.serial.open()
        else:
            self.serial = serial_device
        self.neutral = neutral
        self.session = None
        self.seq = 0
        self.buffer = b""
        self.values = None
        self.updated = 0

    def exchange(self, command, expected, timeout=0.15):
        self.serial.write((command + "\n").encode("ascii"))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.buffer += self.serial.read(min(self.serial.in_waiting, 8192))
            lines = self.buffer.split(b"\n")
            self.buffer = lines.pop()[-8192:]
            found = False
            for line in lines:
                line = line.strip()
                if line == expected.encode():
                    found = True
                else:
                    try:
                        self.values = parse_rc_line(line)
                        self.updated = time.monotonic()
                    except (ValueError, UnicodeError):
                        pass
            if found:
                return
            time.sleep(0.002)
        raise TimeoutError(
            f"ESP32 did not acknowledge {command.split(',')[0]}; firmware watchdog stops driving"
        )

    def connect(self):
        # USB open may reset the ESP32; retry only the non-actuating HELLO handshake.
        deadline = time.monotonic() + 5
        while True:
            try:
                self.exchange("HELLO", "RC_READY,1", 0.3)
                return
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "ESP32 override firmware missing/unresponsive. Upload the updated firmware."
                    )

    def arm(self):
        self.session = secrets.randbelow(2147483646) + 1
        self.exchange(f"ARM,{self.session},{self.neutral[0]},{self.neutral[1]}", f"ACK,ARM,{self.session}")

    def drive(self, steer, throttle):
        if self.session is None:
            raise RuntimeError("Car override is not armed")
        if (
            type(steer) is not int
            or type(throttle) is not int
            or not 0 <= steer <= 255
            or not 50 <= throttle <= 145
        ):
            raise ValueError("Car target outside steering [0,255] / throttle [50,145]")
        self.exchange(f"DRIVE,{self.session},{self.seq},{steer},{throttle}", f"ACK,DRIVE,{self.seq}")
        self.seq += 1

    def stop(self):
        if self.session is not None:
            try:
                self.exchange(f"STOP,{self.session}", f"ACK,STOP,{self.session}")
            finally:
                self.session = None

    def close(self):
        try:
            self.stop()
        finally:
            self.serial.close()


def flash_command(cfg):
    import shutil
    from pathlib import Path

    port = cfg.get("esp32_port")
    if not isinstance(port, str) or not port.startswith("/dev/"):
        raise ValueError("Set the ESP32 /dev/ port in laptop configuration before flashing")
    executable = shutil.which("platformio") or shutil.which("pio")
    if executable is None:
        bundled = Path.home() / ".platformio/penv/bin/platformio"
        if bundled.is_file():
            executable = str(bundled)
    if executable is None:
        raise RuntimeError("PlatformIO is not installed; install it before flashing the ESP32")
    project = Path(__file__).resolve().parents[1] / "esp32_controller"
    return [executable, "run", "-d", str(project), "-t", "upload", "--upload-port", port]


def main():
    from .config import load_config

    parser = argparse.ArgumentParser(description="Restore ESP32 manual control after autonomous inference")
    parser.add_argument("--config", default="config/laptop.local.json")
    parser.add_argument("--flash", action="store_true", help="Upload firmware to the configured ESP32 port")
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.flash:
        import subprocess

        print(f"Flashing ESP32 at {cfg['esp32_port']}; keep the car powered off during upload.", flush=True)
        subprocess.run(flash_command(cfg), check=True)
        return
    car = Car(cfg["esp32_port"], (0, 50))
    try:
        car.connect()
        car.exchange("MANUAL", "ACK,MANUAL")
        print("ESP32 manual control restored.")
    finally:
        car.close()


if __name__ == "__main__":
    main()
