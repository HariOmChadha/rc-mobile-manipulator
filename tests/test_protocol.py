import shutil
import subprocess
from pathlib import Path

import pytest

from mobile_robot.hardware import RCReader
from mobile_robot.protocol import JOINTS, parse_rc_line, validate_action


@pytest.mark.parametrize("line", ["17,203\n", b"17,203\r\n", " 17 , 203 "])
def test_steering_is_first(line):
    assert parse_rc_line(line) == {"steer": 17, "throttle": 203}


@pytest.mark.parametrize("line", ["booting", "1,2,3", "nan,1", "1,inf", "-1,2", "1,256", b"\xff,1", ","])
def test_reject_bad_rc(line):
    with pytest.raises((ValueError, UnicodeError)):
        parse_rc_line(line)


def test_serial_fragmentation_and_last_valid_sample():
    class Serial:
        data = b"17,2"

        @property
        def in_waiting(self):
            return len(self.data)

        def read(self, count):
            result, self.data = self.data[:count], self.data[count:]
            return result

    reader = RCReader()
    reader.serial = Serial()
    assert reader.read()["values"] is None
    reader.serial.data = b"03\nnoise\n25,190\ntrailing"
    assert reader.read()["values"] == {"steer": 25, "throttle": 190}
    reader.serial.data = b"\n"
    assert reader.read()["values"] == {"steer": 25, "throttle": 190}


@pytest.mark.parametrize("value", [True, "0", float("nan"), float("inf"), -101, 101, None])
def test_reject_invalid_joint_values(value):
    target = dict.fromkeys(JOINTS, 0)
    target[JOINTS[0]] = value
    with pytest.raises(ValueError):
        validate_action(target)


def test_joint_schema_and_gripper_range():
    for action in (
        {},
        {"elbow_flex.pos": 3},
        {**dict.fromkeys(JOINTS, 0), "extra.pos": 1},
        {**dict.fromkeys(JOINTS, 0), "gripper.pos": -1},
    ):
        with pytest.raises(ValueError):
            validate_action(action)
    assert validate_action(dict.fromkeys(JOINTS, 100)) == dict.fromkeys(JOINTS, 100.0)


def test_actual_firmware_output_with_distinct_analog_inputs(tmp_path):
    """Execute the checked-in C++ loop with stub ADC/DAC hardware, not a copied formula."""
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ unavailable for firmware harness")
    (tmp_path / "Arduino.h").write_text("""
#pragma once
#include <iostream>
#include <stdexcept>
struct SerialStub {
 void begin(int) {}
 template<class T> void print(T v) { std::cout << v; }
 template<class T> void println(T v) { std::cout << v << "\\n"; }
};
inline SerialStub Serial;
inline void analogReadResolution(int) {}
inline int analogRead(int pin) { return pin == 34 ? 1600 : 3200; }
inline void dacWrite(int pin, int value) {
 if ((pin == 25 && value != 100) || (pin == 26 && value != 200))
   throw std::runtime_error("wrong DAC output");
}
inline void delay(int) {}
""")
    firmware = Path("esp32_controller/src/main.cpp").resolve()
    (tmp_path / "test.cpp").write_text(f'#include "{firmware}"\nint main() {{ setup(); loop(); }}\n')
    binary = tmp_path / "firmware_test"
    subprocess.run(
        [compiler, "-std=c++17", "-I", str(tmp_path), str(tmp_path / "test.cpp"), "-o", str(binary)],
        check=True,
        capture_output=True,
    )
    line = subprocess.check_output([str(binary)], text=True)
    assert line == "100,200\n"
    assert parse_rc_line(line) == {"steer": 100, "throttle": 200}
