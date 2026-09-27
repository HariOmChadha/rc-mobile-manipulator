import json

import pytest

from mobile_robot import setup_inference as setup


def key_file(tmp_path):
    path = tmp_path / "pi-sa-key.json"
    path.write_text(
        json.dumps(
            {
                "type": "service_account",
                "client_email": "test@example.invalid",
                "private_key": "unit-test-not-a-real-key",
                "token_uri": "https://oauth2.googleapis.com/token",
            }
        )
    )
    return path


def test_install_uses_documented_registry_and_keeps_key_out_of_args(tmp_path):
    path = key_file(tmp_path)
    calls = []
    setup.install(str(path), runner=lambda args, **kwargs: calls.append((args, kwargs)))
    assert len(calls) == 2
    command, options = calls[1]
    assert setup.SDK_REQUIREMENT in command
    assert command[command.index("--index-url") + 1] == setup.REGISTRY
    assert command[command.index("--keyring-provider") + 1] == "import"
    assert "--constraint" in command
    assert options["env"]["GOOGLE_APPLICATION_CREDENTIALS"] == str(path)
    assert options["env"]["PYTHON_KEYRING_BACKEND"] == "keyrings.gauth.GooglePythonAuth"
    assert options["check"] is True
    assert not any("unit-test-not-a-real-key" in value or str(path) in value for value in command)


def test_missing_credentials_never_attempt_install(monkeypatch):
    monkeypatch.setattr(setup, "local_setting", lambda name: None)
    calls = []
    with pytest.raises(ValueError, match="Missing PI registry key"):
        setup.install(runner=lambda *args, **kwargs: calls.append(args))
    assert calls == []


@pytest.mark.parametrize(
    "payload",
    [None, {}, {"PI_API_KEY": "not-a-registry-key"}, {"type": "service_account", "private_key": "sensitive"}],
)
def test_bad_registry_file_is_rejected_without_echoing_key(tmp_path, payload):
    path = tmp_path / "key.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError) as error:
        setup.registry_key_path(str(path))
    assert "sensitive" not in str(error.value)


def test_registry_setting_reads_only_requested_env_value(tmp_path, monkeypatch):
    path = key_file(tmp_path)
    monkeypatch.setattr(
        setup,
        "local_setting",
        lambda name: str(path) if name == "GOOGLE_APPLICATION_CREDENTIALS" else "should-not-be-used",
    )
    assert setup.registry_key_path() == path


def test_check_does_not_invoke_network_or_leak_credentials(tmp_path, monkeypatch):
    path = key_file(tmp_path)
    monkeypatch.setattr(setup, "sdk_version", lambda: "0.3.2")
    monkeypatch.setattr(
        setup, "local_setting", lambda name: "private-api-key" if name == "PI_API_KEY" else str(path)
    )
    monkeypatch.setattr(setup.subprocess, "run", lambda *args, **kwargs: pytest.fail("unexpected subprocess"))
    report = setup.status("config/inference.pi-fleet.json")
    assert report["ready_for_model_info"] is True
    assert report["live_model_verified"] is False
    assert report["missing_robot_configuration"]
    assert "private-api-key" not in json.dumps(report)
    assert "unit-test-not-a-real-key" not in json.dumps(report)
