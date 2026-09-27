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

The connected Pi has now been prepared at `/home/pi/rc-mobile-manipulator`
(Debian 13, Python 3.13). Dependencies and arm-driver imports pass, using CPU-only
PyTorch. The private token, follower calibration and local Pi configuration have
been transferred. All 71 tests passed on both the Pi and laptop.

The live Ethernet dummy run passed: 299 control samples in 10 seconds, 29.85 Hz,
1.09 ms p95 request latency, and 598 decoded JPEGs across three cameras (wrist
1280x720, car and scene 640x480). The actual Pi's command-loss watchdog also passed.
The live Wi-Fi run **did not pass**: it saved 455 samples before a request timeout
at about 20 seconds. Median latency was 18.7 ms and p95 was 89.1 ms among completed
requests; these figures exclude the timed-out request. Wi-Fi power saving was
disabled on both devices and the laptop used 5 GHz, but the Pi could only connect
on 2.4 GHz. A stable network still needs validation before wireless motion tests.
Synthetic cameras cannot verify real USB capture, image quality or worst-case
video bandwidth. Detailed local reports are under `data/pi-validation/`.

For this prepared Pi, skip extraction/installation below. Use `./robot devices`
on each machine when USB hardware arrives, confirm the local device assignments,
and follow the hardware checks below. The saved laptop Ethernet fallback is
`config/laptop-ethernet.local.json`; supply it with `--config` to `check` or `record`.

### Preparing another Pi

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
