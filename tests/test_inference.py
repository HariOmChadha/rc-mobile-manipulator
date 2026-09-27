import json
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from mobile_robot.car import Car
from mobile_robot.inference import MockCar, MockPolicy, decode_actions, run, validate_spec
from mobile_robot.protocol import JOINTS
from test_integration import network  # noqa: F401


@pytest.fixture
def spec():
    value = json.loads(Path("config/inference.json").read_text())
    value.update(neutral_steer=128, neutral_throttle=100, mapping_confirmed=True)
    return value


def test_mapping_and_dac_contract(spec):
    validate_spec(spec)
    spec["action_names"] = list(reversed(spec["action_names"]))
    result = decode_actions([[101, 123, 10, 5, 4, 3, 2, 1]], spec)
    assert result == [(dict(zip(JOINTS, [1, 2, 3, 4, 5, 10], strict=True)), (123, 101))]


@pytest.mark.parametrize(
    "row",
    [
        [0] * 7,
        [0] * 9,
        [0] * 6 + [128, 0],
        [0] * 6 + [256, 100],
        [0] * 6 + [128, 146],
        [float("nan")] * 8,
        [True] * 8,
        ["1"] * 8,
        [101] * 6 + [128, 100],
    ],
)
def test_invalid_actions(row, spec):
    with pytest.raises(ValueError):
        decode_actions(row, spec)


@pytest.mark.parametrize(
    "key,value",
    [
        ("mapping_confirmed", False),
        ("neutral_steer", None),
        ("neutral_throttle", 0),
        ("fps", float("nan")),
        ("execution_horizon", 0),
        ("state_names", list(JOINTS)),
    ],
)
def test_unconfirmed_configuration(key, value, spec):
    spec[key] = value
    with pytest.raises(ValueError):
        validate_spec(spec)


def test_inference_camera_arm_car_tcp(network, spec):  # noqa: F811
    cfg, _, path = network
    cars = []

    class CarFactory(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

    class Policy(MockPolicy):
        def infer(self, obs):
            assert obs["state"].shape == (8,)
            for key in spec["camera_map"]:
                assert obs[key].shape == (120, 160, 3)
                assert obs[key].dtype == np.uint8
            time.sleep(0.12)
            return super().infer(obs)

    result = run(
        cfg,
        spec,
        Policy(spec),
        mock=True,
        duration=0.8,
        car_factory=CarFactory,
        output=path / "actions.jsonl",
    )
    assert result["predictions"] >= 2
    assert (123, 101) in cars[0].commands
    assert cars[0].commands.count((128, 100)) >= 3
    assert cars[0].stopped
    assert len((path / "actions.jsonl").read_text().splitlines()) == result["ticks"]


@pytest.mark.parametrize("failure", ["invalid", "timeout", "exception"])
def test_model_failure_stops_both(network, spec, failure):  # noqa: F811
    from mobile_robot.control import Remote

    cfg, _, _ = network
    cars = []

    class CarFactory(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

    class Policy:
        def infer(self, obs):
            if failure == "timeout":
                time.sleep(0.5)
            if failure == "exception":
                raise RuntimeError("model broke")
            return [float("nan")] * 8

    spec["max_inference_s"] = 0.2
    with pytest.raises((ValueError, TimeoutError, RuntimeError)):
        run(cfg, spec, Policy(), mock=True, duration=1, car_factory=CarFactory)
    assert cars[0].stopped
    assert set(cars[0].commands) <= {(128, 100)}
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        state = remote.call("status")
        assert state["session"] is None
        assert state["reason"] == "client_stop"
    finally:
        remote.close()


class Serial:
    def __init__(self):
        self.data = b""
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        # Fragment both ACK and telemetry to test serial framing.
        count = min(count, 4)
        result, self.data = self.data[:count], self.data[count:]
        return result

    def write(self, data):
        self.writes.append(data)
        fields = data.decode().strip().split(",")
        reply = {
            "HELLO": "RC_READY,1",
            "ARM": f"ACK,ARM,{fields[1]}" if len(fields) > 1 else "",
            "DRIVE": f"ACK,DRIVE,{fields[2]}" if len(fields) > 2 else "",
            "STOP": f"ACK,STOP,{fields[1]}" if len(fields) > 1 else "",
        }[fields[0]]
        self.data += b"123,101\n" + reply.encode() + b"\n"

    def close(self):
        self.closed = True


def test_serial_command_ack_and_cleanup():
    serial = Serial()
    car = Car(None, (128, 100), serial_device=serial)
    car.connect()
    car.arm()
    car.drive(123, 101)
    assert car.values == {"steer": 123, "throttle": 101}
    with pytest.raises(ValueError):
        car.drive(123, 146)
    car.close()
    assert serial.closed
    assert [row.split(b",")[0].strip() for row in serial.writes] == [b"HELLO", b"ARM", b"DRIVE", b"STOP"]


def test_missing_ack_fails():
    serial = Serial()
    serial.write = lambda data: None
    car = Car(None, (128, 100), serial_device=serial)
    with pytest.raises(TimeoutError):
        car.exchange("HELLO", "RC_READY,1", 0.01)
    car.close()


def test_firmware_watchdog_and_replay(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ unavailable")
    source = tmp_path / "test.cpp"
    source.write_text(r"""
#include "remote_control.h"
#include <cassert>
#include <string>
int main() {
 RemoteControl r;
 char out[80];
 auto cmd = [&](const char* s, uint32_t now) {r.command(s,now,out,sizeof(out)); return std::string(out);};
 assert(cmd("HELLO",0)=="RC_READY,1");
 assert(cmd("DRIVE,1,0,123,101",0)=="ERR");
 assert(cmd("ARM,1,128,100",10)=="ACK,ARM,1");
 assert(cmd("ARM,2,128,100",10)=="ERR");
 assert(cmd("DRIVE,1,0,123,101",20)=="ACK,DRIVE,0");
 assert(r.steer==123 && r.throttle==101);
 assert(cmd("DRIVE,1,0,250,130",21)=="ERR");
 assert(cmd("DRIVE,2,1,250,130",21)=="ERR");
 assert(cmd("DRIVE,1,1,123,146",21)=="ERR");
 assert(cmd("DRIVE,1,1,123,101,junk",21)=="ERR");
 r.tick(369); assert(r.mode==RemoteControl::AUTO);
 r.tick(370); assert(r.mode==RemoteControl::HOLD && r.steer==128 && r.throttle==100);
 assert(cmd("DRIVE,1,1,123,101",371)=="ERR");
 assert(cmd("ARM,2,128,100",400)=="ACK,ARM,2");
 assert(cmd("STOP,2",401)=="ACK,STOP,2");
 assert(r.mode==RemoteControl::HOLD && r.throttle==100);
 assert(cmd("MANUAL",402)=="ACK,MANUAL");
 assert(r.mode==RemoteControl::MANUAL);
 assert(cmd("ARM,3,128,100",0xffffff00U)=="ACK,ARM,3");
 r.tick(0x60); assert(r.mode==RemoteControl::HOLD); // millis wraparound
}
""")
    binary = tmp_path / "firmware"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            "esp32_controller/include",
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
    )
    subprocess.run([str(binary)], check=True)


def test_ctrl_c_cli_stops_session(network):  # noqa: F811
    import signal
    import sys
    from mobile_robot.control import Remote

    cfg, _, path = network
    config = path / "laptop.json"
    config.write_text(json.dumps(cfg))
    output = path / "actions.jsonl"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "mobile_robot.inference",
            "--mock",
            "--config",
            str(config),
            "--output",
            str(output),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while not output.exists() or len(output.read_text().splitlines()) < 3:
            assert proc.poll() is None
            assert time.monotonic() < deadline
            time.sleep(0.02)
        proc.send_signal(signal.SIGINT)
        stdout, stderr = proc.communicate(timeout=5)
        assert proc.returncode == 0, stderr
        assert json.loads(stdout)["ticks"] >= 3
        remote = Remote(cfg["control_endpoint"], "integration-test")
        try:
            assert remote.call("status")["reason"] == "client_stop"
        finally:
            remote.close()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def test_stale_camera_stops_car(network, spec, monkeypatch):  # noqa: F811
    import mobile_robot.inference as module

    cfg, _, _ = network
    original = module.collect_frames
    cars = []

    class CarFactory(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

    def frames(*args):
        if cars and len(cars[0].commands) >= 3:
            raise RuntimeError("Remote camera wrist is stale")
        return original(*args)

    monkeypatch.setattr(module, "collect_frames", frames)
    with pytest.raises(RuntimeError, match="stale"):
        run(cfg, spec, MockPolicy(spec), mock=True, duration=2, car_factory=CarFactory)
    assert len(cars[0].commands) == 3
    assert cars[0].stopped


def test_dry_run_never_starts_or_commands_actuators(network, spec, monkeypatch):  # noqa: F811
    from mobile_robot.control import Remote
    import mobile_robot.inference as module

    cfg, _, path = network

    def forbidden(*args, **kwargs):
        pytest.fail("Dry run attempted actuator control")

    monkeypatch.setattr(Remote, "start", forbidden)
    monkeypatch.setattr(Remote, "action", forbidden)
    monkeypatch.setattr(module, "Car", forbidden)
    result = run(
        cfg,
        spec,
        MockPolicy(spec),
        mock=True,
        dry_run=True,
        duration=0.3,
        car_factory=forbidden,
        output=path / "dry.jsonl",
    )
    assert result["predictions"] >= 1
    rows = [json.loads(line) for line in (path / "dry.jsonl").read_text().splitlines()]
    assert all(row["dry_run"] for row in rows)
    remote = Remote(cfg["control_endpoint"], "integration-test")
    try:
        assert remote.call("status")["session"] is None
    finally:
        remote.close()


def test_actual_firmware_serial_to_dac_pins(tmp_path):
    """Exercise the real firmware parser, ACKs, pin writes and timeout together."""
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("g++ unavailable")
    (tmp_path / "Arduino.h").write_text(r"""
#pragma once
#include <string>
#include <sstream>
#include <cstdint>
inline uint32_t clockMs=0;
inline int steerOut=-1, throttleOut=-1;
struct SerialStub {
 std::string input;
 std::ostringstream output;
 void begin(int) {}
 int available() {return input.size();}
 int read() {char c=input[0]; input.erase(0,1); return c;}
 template<class T> void print(T value) {output << value;}
 template<class T> void println(T value) {output << value << "\n";}
};
inline SerialStub Serial;
inline unsigned long millis() {return clockMs;}
inline void delay(int ms) {clockMs += ms;}
inline void analogReadResolution(int) {}
inline int analogRead(int pin) {return pin==34 ? 1600 : 3200;}
inline void dacWrite(int pin,int value) {if(pin==25)steerOut=value; if(pin==26)throttleOut=value;}
""")
    source = tmp_path / "firmware.cpp"
    firmware = Path("esp32_controller/src/main.cpp").resolve()
    source.write_text(
        f'#include "{firmware}"\n'
        + r"""
#include <cassert>
int main() {
 setup(); loop(); assert(steerOut==100 && throttleOut==145);
 Serial.input="ARM,1234,128,100\n"; loop();
 assert(steerOut==128 && throttleOut==100);
 Serial.input="DRIVE,1234,0,17,139\n"; loop();
 assert(steerOut==17 && throttleOut==139);
 assert(Serial.output.str().find("ACK,DRIVE,0")!=std::string::npos);
 Serial.input="DRIVE,1234,1,249,51\n"; loop();
 assert(steerOut==249 && throttleOut==51);
 Serial.input="DRIVE,1234,2,0,145\n"; loop();
 assert(steerOut==0 && throttleOut==145);
 Serial.input="DRIVE,1234,3,255,50\n"; loop();
 assert(steerOut==255 && throttleOut==50);
 Serial.input="DRIVE,1234,4,100,146\n"; loop();
 assert(steerOut==255 && throttleOut==50); // rejected out-of-range throttle
 clockMs+=350; loop(); assert(steerOut==128 && throttleOut==100);
 Serial.input="DRIVE,1234,5,17,139\n"; loop();
 assert(steerOut==128 && throttleOut==100); // no replay after timeout
}
"""
    )
    binary = tmp_path / "firmware"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-I",
            str(tmp_path),
            "-I",
            "esp32_controller/include",
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
    )
    subprocess.run([str(binary)], check=True)


def test_inference_phase_switch_discards_old_prediction(network, spec):  # noqa: F811
    cfg, _, _ = network
    prompts = []
    cars = []

    class Keys:
        count = 0

        def start(self):
            pass

        def poll(self):
            self.count += 1
            return self.count == 3

        def close(self):
            pass

    class Policy(MockPolicy):
        def infer(self, obs):
            prompts.append(obs["prompt"])
            time.sleep(0.12)
            result = super().infer(obs)
            if obs["prompt"] == "drive to object":
                result[0][-2:] = [17, 139]
            else:
                result[0][-2:] = [249, 51]
            return result

    class CarFactory(MockCar):
        def __init__(self, *args):
            super().__init__(*args)
            cars.append(self)

    run(cfg, spec, Policy(spec), mock=True, duration=0.6, phase_keys=Keys(), car_factory=CarFactory)
    assert prompts[:2] == ["drive to object", "pick up object"]
    assert (17, 139) not in cars[0].commands
    assert (249, 51) in cars[0].commands
