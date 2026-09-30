# Development and repository layout

Use Python 3.12+ on Linux. From the repository root, activate your environment and run:

```bash
python -m pip install -r requirements-wireless.txt -r requirements-replay.txt -r requirements-test.txt
python -m pytest -q
python -m ruff check mobile_robot scripts tests
python scripts/check_secrets.py
./robot simulate --duration 10
```

Tests use localhost TCP, synthetic cameras, temporary datasets and fault injection.
The firmware tests compile the actual ESP32 control code against stubbed ADC/DAC
hardware (`g++` required). LeRobot adapter tests skip if LeRobot is unavailable;
the real SDK serialization test skips without the private PI SDK. No tests open
physical actuators or call the hosted policy. CI runs the public dependencies only.

| Path | Purpose |
| --- | --- |
| `mobile_robot/` | Pi server, laptop recording, cameras, policy adapters, replay and CLI |
| `esp32_controller/` | PlatformIO firmware and acknowledged DAC control protocol |
| `config/` | Public templates and camera experiments; `*.local.json` stays private |
| `scripts/` | Pi installation/package tools, secret checks and wired fallback |
| `tests/` | Protocol, recording, inference, firmware and replay regression tests |
| `docs/` | Hardware reference, operating guides, project page and demo assets |
| `data/`, `training_dataset/`, `calibration/` | Local output/calibration, excluded from Git |
| `lerobot/`, `.venv/` | Optional local checkout/environment, excluded from Git |

## Cleanup and older entry points

The maintained commands are `./robot pi`, `./robot record`, `./robot infer` and
`./robot replay`. The redundant `pi_onboard/pi_follower.py` and
`laptop_code/laptop_teleop.py` wrappers were removed; use those commands or
`python -m mobile_robot.server` / `python -m mobile_robot.client` instead.

Placeholder loggers, hard-coded serial debug/drive scripts and PlatformIO template
README files were removed. Their history remains in Git. The workstation-specific
udev workaround under `work/` remains local and is no longer tracked. No datasets,
calibrations, installed environments or local configurations were deleted.

The tested direct-USB recorder moved from `laptop_unified/record_episode.py` to
`scripts/record_wired.py`. It is a legacy fallback with fixed ports/camera indices
that must be edited for the attached hardware. It uses an older dataset schema,
has no network watchdog, phase labeling or good/bad review, and is not the Pi
recording path. Do not use its files with the current replay tool without conversion.

## Credentials and publishing

Copy `.env.example` to `.env` for PI credentials. Put registry JSON keys under
`data/credentials/` or outside the checkout. Keep robot tokens in `.robot-token`.
The template contains no credentials. Never put keys in a README, command argument,
screenshot or tracked config. If a credential was exposed, rotate it with its issuer.

Enable the repository's credential guard once per clone:

```bash
git config core.hooksPath .githooks
```

The pre-commit hook scans the exact staged blobs and blocks common secret formats. It is enabled in the original local checkout. Also check manually before publishing:

```bash
python scripts/check_secrets.py --staged
git diff --cached --stat
```

The scanner checks staged Git blobs, not just working files, and reports only paths
and rule names. `--history` additionally scans reachable Git history. This is a
targeted guard for common credentials and private file paths, not a guarantee
against every possible secret format. CI also checks the tracked tree.

`scripts/package_pi.py` intentionally creates a **private** deployment archive with
the shared control token, follower calibration and Pi configuration. It is written
under ignored `data/deployment/` with mode 0600. Transfer it only to your Pi; do not
attach it to a release or commit it. Documentation media lives in `docs/assets/`;
training captures and checkpoint files do not belong there.

## Project page and media

Open `docs/index.html` directly, or preview with:

```bash
python -m http.server 8000 --bind 127.0.0.1 --directory docs
```

Then visit `http://127.0.0.1:8000`. This serves only the public documentation tree,
not the repository containing private files. The supplied demo was transcoded to
720p H.264 without changing the timeline. The README GIF is a reduced-frame-rate
preview; the MP4 retains 30 playback frames/s. Neither proves 30 unique camera
frames/s. The two supplied hardware photos were resized and stripped of metadata.
