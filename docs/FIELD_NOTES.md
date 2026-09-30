# Field notes — September 26–27, 2026

These are historical deployment notes for the original workstation and Pi. Addresses, paths, test counts, and camera assignments below describe that session; they are not current readiness guarantees. New installations should follow [SETUP.md](SETUP.md).

Camera swaps, INNO-MAKER profiles and comparison commands are documented in
[CAMERA_EXPERIMENTS.md](CAMERA_EXPERIMENTS.md).

The laptop environment, local configuration files, shared token and saved
calibrations are prepared. Commands below run from `/home/czarhc/act-athon`.
The `robot` launcher selects the project Python environment and loads the private
token automatically; no activation or manual token copying is needed.

## Switch between Octopus Wi-Fi and Ethernet

On the laptop, stop the current recording, then select the next connection:

```bash
./robot wifi       # Saved Pi address on Octopus
./robot ethernet   # Saved Pi address on the Ethernet cable
./robot network    # Show the selected addresses; does not test connectivity
./robot check      # Check feedback and cameras with the Pi server running
```

Both addresses are prepared in the ignored `config/network.local.json` on this
laptop. These commands change only the control/video endpoints in
`config/laptop.local.json`, preserving your chosen cameras, calibration and
recording settings. They do not switch the computer's Wi-Fi network or interrupt
an existing recording. Connect both devices to Octopus before `./robot wifi`;
plug in Ethernet before `./robot ethernet`. The Pi runs the same `./robot pi`
command for both connections and does not need restarting to change transport.
You can leave Ethernet attached while testing Wi-Fi: the selected Pi Wi-Fi IP
uses the Wi-Fi route. Explicit `--config` arguments override the selected default.

If DHCP changes the Pi address, update just that saved connection:

```bash
./robot wifi --host NEW_PI_WIFI_IP
./robot ethernet --host NEW_PI_ETHERNET_IP
```

For a dummy run, start `./robot pi --mock` on the Pi, then run on the laptop:

```bash
./robot wifi
./robot check --mock
./robot record --mock --duration 10 --label wifi-test
./robot ethernet
./robot check --mock
./robot record --mock --duration 10 --label ethernet-test
```

Stop the dummy Pi server before starting the real one. Camera selection is
independent of transport: change just `wrist`, `car` or `scene` using the commands
in [CAMERA_EXPERIMENTS.md](CAMERA_EXPERIMENTS.md), then use either connection.

## Test now without hardware

```bash
./robot simulate --duration 10
```

This starts and stops both simulated processes, checks the camera streams, records
an episode, decodes every saved JPEG, checks steering/throttle labels and command
sequence, and tests the command-loss watchdog. Each run saves logs, images and a
`report.json` under `data/validation/`. It never opens real USB devices.

## Benchmark connected hardware without movement

On the Pi, run `./robot pi --read-only`. This opens the servo bus for reads only,
compares its saved calibration, and streams the actual cameras. It does not change
motor configuration or torque, and rejects all control-session/action requests.
Leave the arms in their current supported positions.

On the laptop:

```bash
./robot ethernet
./robot benchmark --duration 60 --output data/benchmarks/ethernet
./robot wifi
./robot benchmark --duration 60 --output data/benchmarks/wifi
```

The benchmark reads both arms' six raw encoder positions, ESP32 steering/throttle
telemetry and all three cameras. It saves telemetry and images, decodes every
saved JPEG, and reports sample frequency, request latency, camera capture rate,
image age estimates and saved image bandwidth. Reports are under the requested
output directory, including failure reports. Timeouts and stale frames fail the
test; they are not silently skipped. Calibration matches are reported separately:
raw encoder reads do not need calibrated joint conversion, but a calibration
mismatch must be resolved before normal movement mode. This verifies observation delivery, not
motion tracking or a car emergency stop. Opening the controller serial port
requests DTR/RTS low to avoid resetting it; no control data is written to it.

If the laptop leader/controller are unavailable, add `--skip-local-controls` for
an explicitly partial follower-and-cameras benchmark. Its report marks those
local devices as untested; it is not a full hardware readiness check.

Stop the read-only Pi server before the later movement test and start `./robot pi`
normally. Then run `./robot check` before `./robot record --enable-motion`.

## When the Pi is available

The connected Pi has now been prepared at `/home/pi/rc-mobile-manipulator`
(Debian 13, Python 3.13). Dependencies and arm-driver imports pass, using CPU-only
PyTorch. The private token, follower calibration and local Pi configuration have
been transferred. All 71 tests passed on both the Pi and laptop.

The live Ethernet dummy run passed: 299 control samples in 10 seconds, 29.85 Hz,
1.09 ms p95 request latency, and 598 decoded JPEGs across three cameras (wrist
1280x720, car and scene 640x480). The actual Pi's command-loss watchdog also passed.
The initial actor-labs Wi-Fi run **did not pass**: it saved 455 samples before a request timeout
at about 20 seconds. Median latency was 18.7 ms and p95 was 89.1 ms among completed
requests; these figures exclude the timed-out request. Wi-Fi power saving was
disabled on both devices and the laptop used 5 GHz, but the Pi could only connect
on 2.4 GHz. A later **Octopus ten-second dummy run passed** with both devices on
2.4 GHz: 267 samples (26.7 Hz), 10.8 ms median and 46.0 ms p95 request latency,
with 513 decoded JPEGs. This is a short connectivity test, not proof of sustained
30 Hz or reliable operation with real cameras. Longer hardware validation remains.
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

For each subsequent recording, keep the Pi server running and use this on the laptop:

```bash
cd /home/czarhc/act-athon && ./robot record --enable-motion
```

Recording starts in **drive to object**. Press **Enter** to advance to **pick up
object**, then **drive to the bin**, then **drop in bin**. The terminal shows the
active phase, and every telemetry row records it. Enter in the final phase keeps
recording there. Keep this terminal focused for phase keys.

Press Ctrl+C to finish and save, then enter `g` for good or `b` for bad. Episodes go
to `training_dataset/good/episode_...` or `training_dataset/bad/episode_...` on the
laptop. Enter alone leaves the run in `training_dataset/unreviewed/episode_...`.
The prompt appears after arm-session shutdown and file flushing. Repeat the same
command for a new episode. Earlier recordings in `data/episode_*` are retained;
use `./robot classify EPISODE_PATH good` (or `bad`) to file one later.

For a staged arm-only test, temporarily set `cameras: []` in both local configs
and `remote_cameras: []` in the laptop config. Restore one camera at a time.

Rebuild the Pi bundle after any code or Pi configuration changes:

```bash
python3 scripts/package_pi.py
```

See [SETUP.md](SETUP.md) for calibration instructions, dataset semantics and limits.
