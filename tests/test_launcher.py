import json
import sys

import pytest

from mobile_robot import launch


def prepare(tmp_path, monkeypatch, endpoint="tcp://PI_ADDRESS:5555"):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("ROBOT_TOKEN", raising=False)
    (tmp_path / "config").mkdir()
    cfg = {
        "control_endpoint": endpoint,
        "video_endpoint": "tcp://PI_ADDRESS:5556",
        "leader": {"port": "/dev/serial/by-id/example"},
        "cameras": [],
    }
    (tmp_path / "config/laptop.local.json").write_text(json.dumps(cfg))
    return cfg


def test_setting_pi_preserves_device_choices(tmp_path, monkeypatch):
    cfg = prepare(tmp_path, monkeypatch)
    monkeypatch.setattr(sys, "argv", ["robot", "set-pi", "robot-pi.local"])
    launch.main()
    result = json.loads((tmp_path / "config/laptop.local.json").read_text())
    assert result["control_endpoint"] == "tcp://robot-pi.local:5555"
    assert result["video_endpoint"] == "tcp://robot-pi.local:5556"
    assert result["leader"] == cfg["leader"]
    assert result["cameras"] == []


def test_unconfigured_pi_does_not_launch_hardware_client(tmp_path, monkeypatch):
    prepare(tmp_path, monkeypatch)
    monkeypatch.setattr(sys, "argv", ["robot", "check"])
    monkeypatch.setattr(launch.os, "execv", lambda *args: pytest.fail("Client must not start"))
    with pytest.raises(SystemExit) as error:
        launch.main()
    assert error.value.code == 2


def test_record_does_not_silently_enable_motion_or_expose_secret(tmp_path, monkeypatch):
    prepare(tmp_path, monkeypatch, "tcp://robot-pi.local:5555")
    (tmp_path / ".robot-token").write_text("private-test-token\n")
    monkeypatch.setattr(sys, "argv", ["robot", "record", "--duration", "5"])
    calls = []
    monkeypatch.setattr(launch.os, "execv", lambda *args: calls.append(args))
    launch.main()
    assert "--enable-motion" not in calls[0][1]
    assert "private-test-token" not in str(calls)
    assert launch.os.environ["ROBOT_TOKEN"] == "private-test-token"


def test_explicit_token_takes_precedence(tmp_path, monkeypatch):
    prepare(tmp_path, monkeypatch)
    (tmp_path / ".robot-token").write_text("file-token\n")
    monkeypatch.setenv("ROBOT_TOKEN", "environment-token")
    monkeypatch.setattr(sys, "argv", ["robot", "devices"])
    monkeypatch.setattr(launch.os, "execv", lambda *args: None)
    launch.main()
    assert launch.os.environ["ROBOT_TOKEN"] == "environment-token"


def test_experiment_config_overrides_unconfigured_default(tmp_path, monkeypatch):
    cfg = prepare(tmp_path, monkeypatch)
    cfg["control_endpoint"] = "tcp://robot-pi.local:5555"
    variant = tmp_path / "variant.json"
    variant.write_text(json.dumps(cfg))
    monkeypatch.setattr(sys, "argv", ["robot", "check", "--config", str(variant)])
    calls = []
    monkeypatch.setattr(launch.os, "execv", lambda *args: calls.append(args))
    launch.main()
    assert calls[0][1][-2:] == ["--config", str(variant)]
