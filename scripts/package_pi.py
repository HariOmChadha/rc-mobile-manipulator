"""Package this working tree for the Pi, including its private local settings."""

import hashlib
import tarfile
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    files = [
        root / name
        for name in (
            "robot",
            "README.md",
            "QUICKSTART.md",
            "CAMERA_EXPERIMENTS.md",
            "requirements-wireless.txt",
            ".robot-token",
            "config/pi.json",
            "config/pi.local.json",
            "config/laptop.json",
            "config/network.json",
            "config/camera_profiles.json",
            "calibration/follower/my_follower_arm.json",
            "scripts/setup_pi.sh",
        )
    ]
    files.extend(sorted((root / "mobile_robot").glob("*.py")))
    files.extend(sorted((root / "config" / "experiments").glob("pi-*.json")))
    for path in files:
        if not path.is_file():
            raise FileNotFoundError(f"Missing preparation file: {path}")
    output = root / "data" / "deployment" / "pi-ready.tar.gz"
    output.parent.mkdir(parents=True, exist_ok=True)
    # Restrict the archive before writing: it contains the shared control token.
    with output.open("wb") as stream:
        output.chmod(0o600)
        with tarfile.open(fileobj=stream, mode="w:gz") as archive:
            for path in files:
                archive.add(path, arcname=str(Path("rc-mobile-manipulator") / path.relative_to(root)))
    with tarfile.open(output) as archive:
        for path in files:
            name = str(Path("rc-mobile-manipulator") / path.relative_to(root))
            if archive.extractfile(name).read() != path.read_bytes():
                raise RuntimeError(f"Bundle verification failed for {name}")
    checksum = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(".gz.sha256").write_text(f"{checksum}  {output.name}\n")
    print(f"Verified Pi bundle: {output} ({len(files)} files)")


if __name__ == "__main__":
    main()
