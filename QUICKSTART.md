# Prepared setup

Camera swaps, INNO-MAKER profiles and comparison commands are documented in
[CAMERA_EXPERIMENTS.md](CAMERA_EXPERIMENTS.md).

The laptop environment, local configuration files, shared token and saved
calibrations are prepared. Commands below run from `/home/czarhc/act-athon`.
The `robot` launcher selects the project Python environment and loads the private
token automatically; no activation or manual token copying is needed.

## Test now without hardware

```bash
./robot simulate --duration 10
```

This starts and stops both simulated processes, checks the camera streams, records
an episode, decodes every saved JPEG, checks steering/throttle labels and command
sequence, and tests the command-loss watchdog. Each run saves logs, images and a
`report.json` under `data/validation/`. It never opens real USB devices.

## When the Pi is available

`data/deployment/pi-ready.tar.gz` contains the current code, the Pi config, follower
calibration and matching private control token. Transfer it only to your Pi.
It does not contain a laptop virtual environment or preinstalled ARM dependencies.

On the Pi, extract it into a new directory, then install the pinned dependencies:

```bash
tar -xzf pi-ready.tar.gz
cd rc-mobile-manipulator
bash scripts/setup_pi.sh
./robot devices
```

The installation requires Internet access, Git, and Python 3.12+ with venv support.
If that Python is named differently, run
`ROBOT_PYTHON=python3.12 bash scripts/setup_pi.sh`.
Confirm the follower serial path and two camera paths in `config/pi.local.json`.
The saved calibration is ready, but still needs to match the physical arm's
motor calibration. Support the arm before connecting; startup can enable torque.
Then start the Pi host:

```bash
./robot pi
```

## Laptop connection and short hardware run

```bash
./robot devices
./robot set-pi YOUR_PI_IP
```

Confirm `leader.port`, `esp32_port` and the scene camera path in
`config/laptop.local.json`. Their current values are the previous USB defaults,
not verified assignments. The only camera present during preparation was the
laptop webcam, so it has **not** been confirmed as the scene camera.

With the follower supported, the workspace clear and the leader close to the
follower's current pose:

```bash
./robot check
./robot record --enable-motion --duration 5
```

The first command checks follower feedback and all three camera streams without
sending motion commands. The second initializes the leader and ESP32, then runs
one five-second episode and ends the arm session. If checks fail, fix the reported
device/calibration/network issue before retrying. The arm watchdog does not stop
the separately controlled RC car.

For a staged arm-only test, temporarily set `cameras: []` in both local configs
and `remote_cameras: []` in the laptop config. Restore one camera at a time.

Rebuild the Pi bundle after any code or Pi configuration changes:

```bash
python3 scripts/package_pi.py
```

See `README.md` for full calibration instructions, dataset semantics and limits.
