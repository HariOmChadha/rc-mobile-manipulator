import json
import sys

import pytest

from mobile_robot import launch, network


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("ROBOT_TOKEN", raising=False)
    (tmp_path / "config").mkdir()
    profiles = {"wifi": {"host": "172.20.10.13"}, "ethernet": {"host": "10.42.0.181"}}
    (tmp_path / "config/network.json").write_text(json.dumps(profiles))
    cfg = {
        "control_endpoint": "tcp://10.42.0.181:5555",
        "video_endpoint": "tcp://10.42.0.181:5556",
        "leader": {"port": "custom-serial", "calibration_dir": "../calibration/leader"},
        "cameras": [{"name": "scene", "profile": "innomaker-u20cam-720p", "device": "custom-camera"}],
        "remote_cameras": ["wrist", "car"],
        "request_timeout_s": 0.3,
        "output_dir": "my-experiment",
    }
    (tmp_path / "config/laptop.local.json").write_text(json.dumps(cfg))
    return cfg


def test_switch_both_transports_preserves_camera_experiment(setup, monkeypatch):
    from pathlib import Path

    for mode, host in (("wifi", "172.20.10.13"), ("ethernet", "10.42.0.181")):
        monkeypatch.setattr(sys, "argv", ["robot", mode])
        launch.main()
        actual = json.loads(Path("config/laptop.local.json").read_text())
        expected = {**setup, "control_endpoint": f"tcp://{host}:5555", "video_endpoint": f"tcp://{host}:5556"}
        assert actual == expected


def test_changed_dhcp_address_is_remembered_without_changing_other_transport(setup, capsys):
    from pathlib import Path

    network.main(["wifi", "--host", "172.20.10.7"])
    network.main(["ethernet"])
    network.main(["wifi"])
    saved = json.loads(Path("config/network.local.json").read_text())
    assert saved == {"wifi": {"host": "172.20.10.7"}, "ethernet": {"host": "10.42.0.181"}}
    assert (
        json.loads(Path("config/laptop.local.json").read_text())["video_endpoint"] == "tcp://172.20.10.7:5556"
    )
    network.main([])
    assert "Selected connection: wifi" in capsys.readouterr().out


@pytest.mark.parametrize("host", ["PI_WIFI_ADDRESS", "bad/host", "--option", "", "host:5555"])
def test_invalid_address_does_not_modify_active_configuration(setup, host):
    from pathlib import Path

    before = Path("config/laptop.local.json").read_bytes()
    with pytest.raises(SystemExit) as error:
        network.main(["wifi", f"--host={host}"])
    assert error.value.code == 2
    assert Path("config/laptop.local.json").read_bytes() == before
    assert not Path("config/network.local.json").exists()


def test_status_reports_custom_endpoints_without_overwriting_them(setup, capsys):
    from pathlib import Path

    setup["video_endpoint"] = "tcp://different-host:6000"
    path = Path("config/laptop.local.json")
    path.write_text(json.dumps(setup))
    before = path.read_bytes()
    network.main([])
    assert "Selected connection: custom" in capsys.readouterr().out
    assert path.read_bytes() == before
