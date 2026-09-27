import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from mobile_robot.inference import decode_actions
from mobile_robot.pi_fleet import ACTION_TOPICS, ARM_STATE, DRIVE_STATE, GRIPPER_STATE, Policy, load_api_key
from mobile_robot.protocol import JOINTS


@pytest.fixture
def spec():
    cfg = json.loads(Path("config/inference.pi-fleet.json").read_text())
    cfg.update(
        camera_map={"observation/image/overhead": "scene", "observation/image/wrist": "wrist"},
        training_rc_neutral={"steer": 120.5, "throttle": 99.5},
        neutral_steer=120,
        neutral_throttle=100,
        mapping_confirmed=True,
    )
    return cfg


@pytest.fixture
def sdk(monkeypatch):
    monkeypatch.setenv("PI_API_KEY", "test-only")

    class Client:
        instances = []

        def __init__(self, settings):
            self.settings = settings
            self.server_metadata = SimpleNamespace(
                camera_names=["observation/image/overhead", "observation/image/wrist"],
                action_keys=list(ACTION_TOPICS),
                action_horizon=50,
            )
            self.actions = {
                ACTION_TOPICS[0]: np.tile([1, 2, 3, 4, 5], (3, 1)),
                ACTION_TOPICS[1]: np.full((3, 1), 6),
                ACTION_TOPICS[2]: np.tile([10 / 127.5, -5 / 127.5], (3, 1)),
            }
            self.closed = False
            self.__class__.instances.append(self)

        def infer(self, obs):
            self.observed = obs
            return SimpleNamespace(actions_by_key=self.actions, server_time_ms=120)

        def close(self):
            self.closed = True

    return SimpleNamespace, SimpleNamespace, Client


def obs(spec):
    return {
        "state": np.asarray([1, 2, 3, 4, 5, 6, 130.5, 94.5]),
        "prompt": "pick up the box",
        **{name: np.full((16, 24, 3), 42, np.uint8) for name in spec["camera_map"]},
    }


def test_hosted_state_scaling_and_output_order(spec, sdk):
    spec["action_names"] = list(reversed(spec["action_names"]))
    policy = Policy(checkpoint=spec["checkpoint"], config=spec, types=sdk)
    result = policy.infer(obs(spec))
    client = sdk[2].instances[-1]
    assert client.settings.gateway_url == spec["checkpoint"]
    assert client.settings.api_key == "test-only"
    assert client.settings.execute_chunk_size == 10
    assert client.settings.action_keys == list(ACTION_TOPICS)
    np.testing.assert_allclose(client.observed.states[DRIVE_STATE], [10 / 127.5, -5 / 127.5])
    np.testing.assert_equal(client.observed.states[ARM_STATE], [1, 2, 3, 4, 5])
    np.testing.assert_equal(client.observed.states[GRIPPER_STATE], [6])
    np.testing.assert_allclose(result[0], [94.5, 130.5, 6, 5, 4, 3, 2, 1])
    # Validated robot-side command remains raw DAC after conversion, not model offsets.
    arm, dac = decode_actions(result, spec)[0]
    assert arm == dict(zip(JOINTS, [1, 2, 3, 4, 5, 6], strict=True))
    assert dac == (130, 94)
    policy.close()
    assert client.closed


def test_reordered_state(spec, sdk):
    policy = Policy(checkpoint=spec["checkpoint"], config=spec, types=sdk)
    data = obs(spec)
    data["state"] = data["state"][::-1]
    spec["state_names"] = list(reversed(spec["state_names"]))
    policy.infer(data)
    np.testing.assert_allclose(sdk[2].instances[-1].observed.states[ARM_STATE], [1, 2, 3, 4, 5])
    policy.close()


@pytest.mark.parametrize("bad", ["missing", "shape", "length", "nan", "bool", "out_of_range"])
def test_invalid_model_output(spec, sdk, bad):
    policy = Policy(checkpoint=spec["checkpoint"], config=spec, types=sdk)
    client = sdk[2].instances[-1]
    if bad == "missing":
        del client.actions[ACTION_TOPICS[2]]
    elif bad == "shape":
        client.actions[ACTION_TOPICS[2]] = np.ones((3, 3))
    elif bad == "length":
        client.actions[ACTION_TOPICS[2]] = np.ones((2, 2))
    elif bad == "nan":
        client.actions[ACTION_TOPICS[2]] = np.full((3, 2), np.nan)
    elif bad == "bool":
        client.actions[ACTION_TOPICS[2]] = np.ones((3, 2), dtype=bool)
    else:
        client.actions[ACTION_TOPICS[2]] = np.ones((3, 2))
    with pytest.raises(ValueError):
        decode_actions(policy.infer(obs(spec)), spec)
    policy.close()


def test_no_black_frames_for_missing_camera(spec, sdk):
    spec["camera_map"] = {"wrong-name": "scene"}
    with pytest.raises(ValueError, match="exact server"):
        Policy(checkpoint=spec["checkpoint"], config=spec, types=sdk)
    assert sdk[2].instances[-1].closed


@pytest.mark.parametrize(
    "neutral", [None, {"steer": 0}, {"steer": float("nan"), "throttle": 100}, {"steer": 0, "throttle": 0}]
)
def test_missing_or_mismatched_training_neutral(spec, sdk, neutral):
    spec["training_rc_neutral"] = neutral
    with pytest.raises(ValueError):
        Policy(checkpoint=spec["checkpoint"], config=spec, types=sdk)
    assert not sdk[2].instances


def test_api_key_file_does_not_execute_or_override(tmp_path, monkeypatch):
    monkeypatch.delenv("PI_API_KEY", raising=False)
    path = tmp_path / ".env"
    path.write_text("UNRELATED=ignored\nexport PI_API_KEY='literal-$(touch nope)'\n")
    assert load_api_key(path) == "literal-$(touch nope)"
    assert not (tmp_path / "nope").exists()
    monkeypatch.setenv("PI_API_KEY", "shell-wins")
    assert load_api_key(path) == "shell-wins"
    monkeypatch.delenv("PI_API_KEY")
    path.write_text("PI_API_KEY=\n")
    with pytest.raises(ValueError, match="Set PI_API_KEY"):
        load_api_key(path)


def test_info_only_closes_connection(spec, sdk, monkeypatch):
    import mobile_robot.pi_fleet as module

    monkeypatch.setattr(module, "sdk_types", lambda: sdk)
    result = module.inspect(checkpoint=spec["checkpoint"], config=spec)
    assert result["unmapped_model_cameras"] == []
    assert result["action_horizon"] == 50
    assert sdk[2].instances[-1].closed
    assert not hasattr(sdk[2].instances[-1], "observed")


def test_flash_targets_configured_esp(monkeypatch):
    from mobile_robot.car import flash_command

    monkeypatch.setattr("shutil.which", lambda name: "/test/platformio")
    command = flash_command({"esp32_port": "/dev/serial/by-id/esp32"})
    assert command[-2:] == ["--upload-port", "/dev/serial/by-id/esp32"]
    assert command[command.index("-t") + 1] == "upload"
