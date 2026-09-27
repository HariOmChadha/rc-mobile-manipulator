"""Select one named camera, preserving other cameras and arm/network settings."""

import argparse
import json
import os
from pathlib import Path

from .config import load_config
from .profiles import catalog


def set_camera(source, output, name, profile=None, **changes):
    source, output = Path(source).resolve(), Path(output).resolve()
    cfg = json.loads(source.read_text())
    matches = [camera for camera in cfg.get("cameras", []) if camera["name"] == name]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one camera named {name!r} in {source}")
    camera = matches[0]
    if profile:
        profiles = catalog()
        if profile not in profiles:
            raise ValueError(f"Unknown camera profile {profile!r}")
        # Remove previous model/mode defaults; preserve role and physical device path.
        keys = {key for preset in profiles.values() for key in preset}
        for key in keys:
            camera.pop(key, None)
        camera["profile"] = profile
    camera.update({key: value for key, value in changes.items() if value is not None})
    # Preserve the meaning of relative calibration paths when saving a variant elsewhere.
    for role in ("leader", "follower"):
        calibration = cfg.get(role, {}).get("calibration_dir")
        if calibration and not Path(calibration).is_absolute():
            absolute = (source.parent / calibration).resolve()
            cfg[role]["calibration_dir"] = os.path.relpath(absolute, output.parent)
    output.parent.mkdir(parents=True, exist_ok=True)
    temp = output.with_name(output.name + ".tmp")
    try:
        temp.write_text(json.dumps(cfg, indent=2) + "\n")
        load_config(temp)  # Validate before replacing the active file.
        temp.replace(output)
    finally:
        temp.unlink(missing_ok=True)
    return output


def main():
    parser = argparse.ArgumentParser(description="List camera profiles or change one named camera slot")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    show = sub.add_parser("show")
    show.add_argument("--config", required=True)
    select = sub.add_parser("set")
    select.add_argument("--config", required=True)
    select.add_argument("--output", help="Save a separate experiment config instead of editing the source")
    select.add_argument("--camera", required=True, help="Existing slot: wrist, car or scene")
    select.add_argument("--profile", choices=list(catalog()))
    select.add_argument("--device", help="Stable /dev/v4l/by-id or by-path node, or integer index")
    for key in ("width", "height", "fps", "jpeg-quality"):
        select.add_argument(f"--{key}", type=int)
    select.add_argument("--fourcc")
    select.add_argument("--label", help="Camera/mount notes stored with every frame")
    select.add_argument("--strict-mode", action=argparse.BooleanOptionalAction, default=None)
    test = sub.add_parser("probe", help="Test cameras on this host; never connects an arm")
    test.add_argument("--config", required=True)
    test.add_argument("--duration", type=float, default=5)
    test.add_argument("--output", default="data/camera-tests")
    test.add_argument("--label")
    test.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    if args.command == "list":
        print(json.dumps(catalog(), indent=2))
    elif args.command == "show":
        print(json.dumps(load_config(args.config)["cameras"], indent=2))
    elif args.command == "probe":
        from .camera_probe import probe

        if not 1 <= args.duration <= 300:
            parser.error("--duration must be between 1 and 300 seconds")
        print(probe(args.config, args.duration, args.output, mock=args.mock, label=args.label))
    else:
        changes = {
            key: getattr(args, key)
            for key in ("device", "width", "height", "fps", "jpeg_quality", "fourcc", "label", "strict_mode")
        }
        if changes["device"] is not None and changes["device"].isdigit():
            changes["device"] = int(changes["device"])
        try:
            output = set_camera(args.config, args.output or args.config, args.camera, args.profile, **changes)
        except (ValueError, OSError) as error:
            parser.exit(1, f"Camera configuration error: {error}\n")
        print(f"Saved {output}. Restart the process on the camera's host to apply it.")


if __name__ == "__main__":
    main()
