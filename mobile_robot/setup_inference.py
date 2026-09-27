"""Install the PI inference SDK from its authenticated registry; never touches robot hardware."""

import argparse
import importlib.metadata
import json
import os
import subprocess
import sys
from pathlib import Path

from .pi_fleet import local_setting, sdk_types

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = "https://us-east5-python.pkg.dev/pi-external-partners/pi-sdk/simple/"
AUTH_PACKAGES = ("keyring==25.7.0", "keyrings.google-artifactregistry-auth==1.1.2")
SDK_REQUIREMENT = "pi-sdk[video,inference]>=0.3.2"


def registry_key_path(value=None):
    value = value or local_setting("GOOGLE_APPLICATION_CREDENTIALS")
    if not value:
        raise ValueError(
            "Missing PI registry key. Obtain the service-account JSON from PI onboarding and set "
            "GOOGLE_APPLICATION_CREDENTIALS=/absolute/path/to/pi-sa-key.json in .env, "
            "or pass --registry-key PATH. This is separate from PI_API_KEY."
        )
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    try:
        payload = json.loads(path.read_text())
    except (OSError, ValueError):
        raise ValueError(
            "Registry credential path must point to a readable service-account JSON file"
        ) from None
    if (
        not isinstance(payload, dict)
        or payload.get("type") != "service_account"
        or any(
            not isinstance(payload.get(key), str) or not payload[key].strip()
            for key in ("client_email", "private_key", "token_uri")
        )
    ):
        raise ValueError(
            "Registry JSON must be the PI-issued service-account key, not the organization API key"
        )
    return path.resolve()


def sdk_version():
    try:
        version = importlib.metadata.version("pi-sdk")
    except importlib.metadata.PackageNotFoundError:
        return None
    from packaging.version import Version

    if Version(version) < Version("0.3.2"):
        return None
    sdk_types()
    return version


def install(registry_key=None, *, runner=None):
    """Pass credential paths through the environment, never put credential contents in pip arguments."""
    if sys.version_info < (3, 11):
        raise RuntimeError("PI SDK requires Python 3.11 or later")
    key_path = registry_key_path(registry_key)
    runner = runner or subprocess.run
    env = dict(os.environ)
    env["GOOGLE_APPLICATION_CREDENTIALS"] = str(key_path)
    # Select only the documented registry backend; avoid desktop keychain prompts.
    env["PYTHON_KEYRING_BACKEND"] = "keyrings.gauth.GooglePythonAuth"
    common = [sys.executable, "-m", "pip", "install", "--no-input"]
    runner([*common, *AUTH_PACKAGES], check=True, env=env, cwd=ROOT)
    runner(
        [
            *common,
            "--keyring-provider",
            "import",
            SDK_REQUIREMENT,
            "--index-url",
            REGISTRY,
            "--extra-index-url",
            "https://pypi.org/simple",
            "--constraint",
            str(ROOT / "requirements-wireless.txt"),
        ],
        check=True,
        env=env,
        cwd=ROOT,
    )


def status(config_path, registry_key=None):
    try:
        registry_key_path(registry_key)
        registry_available = True
    except ValueError:
        registry_available = False
    try:
        version = sdk_version()
    except (ImportError, RuntimeError):
        version = None
    report = {
        "python": sys.version.split()[0],
        "pi_sdk_version": version,
        "registry_key_available": registry_available,
        "api_key_configured": bool(local_setting("PI_API_KEY")),
        "missing_robot_configuration": [],
    }
    cfg = json.loads(Path(config_path).read_text())
    try:
        from .inference import validate_spec

        validate_spec(cfg)
    except (ValueError, KeyError, TypeError) as error:
        report["missing_robot_configuration"].append(str(error))
    if cfg.get("training_rc_neutral") is None:
        report["missing_robot_configuration"].append("Training rc_neutral values are missing")
    report["ready_for_model_info"] = bool(version and report["api_key_configured"])
    # Configuration checks do not imply that the model, mapping, or physical robot was validated.
    report["live_model_verified"] = False
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry-key", help="Path to PI-provided service-account JSON, not an API key")
    parser.add_argument(
        "--check", action="store_true", help="Report local readiness without network or installation"
    )
    parser.add_argument("--inference-config", default=str(ROOT / "config/inference.local.json"))
    args = parser.parse_args()
    try:
        if not args.check:
            install(args.registry_key)
            if not sdk_version():
                raise RuntimeError("SDK installation did not provide pi-sdk >=0.3.2 with inference support")
        report = status(args.inference_config, args.registry_key)
        print(json.dumps(report, indent=2))
        if report["ready_for_model_info"]:
            print("Next: ./robot infer --info (model metadata only; no robot motion)")
        elif not report["api_key_configured"]:
            print("Get the organization API key at https://partner.pi-fleet.com and save PI_API_KEY in .env.")
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Inference setup error: {error}\n")


if __name__ == "__main__":
    main()
