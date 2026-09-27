# RC mobile manipulator

For the already prepared local environment and deployment bundle, start with
[QUICKSTART.md](QUICKSTART.md). `./robot simulate --duration 10` performs a complete
hardware-free check in one command.

An SO101 leader stays on the laptop. An SO101 follower and two USB cameras connect
to the Raspberry Pi on the car. The laptop receives joint feedback and camera
images over Wi-Fi, adds its local scene camera and handheld-controller telemetry,
and records an episode. No `socat` or virtual USB/serial ports are required.

The RC car continues to use its existing handheld radio controller. The checked-in
ESP32 firmware reads the handheld inputs, writes its DAC outputs to the remote PCB,
and reports **steering,throttle** over USB. Recording uses manual control. The new `./robot infer` path can command DAC outputs
through updated ESP32 firmware; see [INFERENCE.md](INFERENCE.md) for setup and limits.
The arm watchdog is separate from the ESP32 watchdog.

## Try it now, without hardware

Run all commands from this repository's root. The laptop already has a working
Python environment at `lerobot/act-athon`:

```bash
source lerobot/act-athon/bin/activate
python -m pip install -r requirements-wireless.txt -r requirements-test.txt
```

Alternatively create a Python 3.12 environment. Simulation needs only
`requirements-wireless.txt`, not LeRobot, CUDA, an ESP32, cameras, or USB devices.

Terminal 1 — simulated Pi:

```bash
python -m mobile_robot.server --mock
```

Terminal 2 — check the connection, then record ten seconds:

```bash
python -m mobile_robot.client --mock --check
python -m mobile_robot.client --mock --duration 10 --output data/simulation
```

The simulation uses **real TCP networking and JPEG encoding**, with two synthetic
Pi cameras and one synthetic laptop camera. The follower model clamps each step
and returns measured-model state separately from requested targets. It does not
model mechanical dynamics, USB timing or Wi-Fi congestion. Output metadata marks
every simulated episode and frame as simulated. Real and simulated control modes
cannot be mixed.

The previous entry points still work with the same arguments:

```bash
python pi_onboard/pi_follower.py --mock
python laptop_code/laptop_teleop.py --mock --duration 10
```

## Prepare the Pi

Use a 64-bit OS with Python **3.12 or newer** supported by the pinned LeRobot
revision. A Pi model has not yet been specified or tested. Install the same source
revision on both computers; camera and arm I/O run on the Pi, but the normal
LeRobot installation still includes its ML dependencies. No training or inference
runs on the Pi in this pipeline.

On a fresh machine, after installing Git, Python 3.12 and its venv support:

```bash
git clone https://github.com/HariOmChadha/rc-mobile-manipulator.git
cd rc-mobile-manipulator
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-wireless.txt
python -m pip install 'lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@e595b7902714ba51f91e47523f66f89c5181b649'
```

These new files must first be copied or committed/pushed to the repository used
on the Pi; cloning the old upstream commit will not include local changes.
The revision above matches the laptop checkout used to validate the adapter.
Do not copy an x86 laptop virtual environment to an ARM Pi. The ARM64 dependency
installation itself remains to be verified on your particular Pi/OS.

For the existing laptop environment, keep its matching editable LeRobot checkout
and install only the wireless requirements. If USB access is denied, check serial
and video group membership (`dialout` and `video` on many Linux distributions);
log out/in after changing groups. Avoid running teleoperation as root.

## Configure the actual devices

Connect the follower and **two** cameras to the Pi. Keep the leader, controller
ESP32 and static scene camera on the laptop. Ensure appropriate regulated power
for the Pi, USB cameras and the arm; USB power does not replace servo power.

On each computer:

```bash
python -m mobile_robot.devices
cp config/pi.json config/pi.local.json
cp config/laptop.json config/laptop.local.json
```

Only edit the Pi config on the Pi and laptop config on the laptop:

| Setting | What to change |
| --- | --- |
| Pi `follower.port` | Follower serial path, preferably `/dev/serial/by-id/...` |
| Pi `cameras[].device` | Two real capture devices, preferably `/dev/v4l/by-id/...-video-index0` or `by-path` |
| Laptop `leader.port` | Leader serial path |
| Laptop `esp32_port` | Handheld ESP32 serial path; `null` explicitly disables this telemetry |
| Laptop `cameras[0].device` | Static scene camera |
| Laptop `control_endpoint` | `tcp://PI_IP:5555` |
| Laptop `video_endpoint` | `tcp://PI_IP:5556` |

The checked-in camera indices are **examples**, not hardware discovery results.
USB webcams may expose extra metadata nodes. Camera names (`wrist`, `car`,
`scene`) must stay distinct and the laptop's `remote_cameras` must match the Pi.
To bench-test only the arm, set `cameras` to `[]` on both computers and
`remote_cameras` to `[]` on the laptop. Add one camera, then the second, then scene.

Start with 640×480, MJPEG, 15 camera frames/s and 30 arm updates/s. Camera drivers
may negotiate a different mode; `--check` reports actual received dimensions.
Use `v4l2-ctl --list-formats-ext -d /dev/videoX` when a camera fails to deliver
MJPEG at the requested mode. A powered hub may be needed depending on USB power
and bandwidth. Use a local Wi-Fi network allowing communication between clients;
guest networks often isolate them. Prefer 5 GHz for the Pi/laptop if supported.

## Calibration

Both sides explicitly use LeRobot normalized joint units: -100..100 for arm
joints, 0..100 for the gripper. Do not mix this with an older degree-based dataset.
`max_relative_target` defaults to 5 normalized units per accepted command; it is a
step limit, not a physical speed or collision guarantee.

The configured calibration paths are relative to the config file. Copy the
**existing calibration for each physical arm** into:

```text
calibration/leader/my_leader_arm.json       # on the laptop
calibration/follower/my_follower_arm.json   # on the Pi
```

If you need to locate the old files, inspect the LeRobot calibration directory
under `~/.cache/huggingface/lerobot/calibration/`. Preserve each arm's own file.
Calibration files are ignored by Git. If none exists or the motor calibration
does not match, use the explicit interactive setup commands:

```bash
# Pi — support the arm and follow LeRobot's prompts
python -m mobile_robot.calibrate --config config/pi.local.json --role follower

# Laptop
python -m mobile_robot.calibrate --config config/laptop.local.json --role leader
```

This command reuses valid existing calibration; it is not a forced recalibration
of an already matching arm. Normal server/client startup never waits for an
unexpected calibration prompt. Connecting the follower configures its motors and
can enable holding torque; have the arm supported and clear before starting it.

## Start with hardware

Set the same nonempty secret in both terminals (keep it out of committed configs):

```bash
export ROBOT_TOKEN='replace-with-the-same-random-secret-on-both-computers'
```

On the Pi:

```bash
python -m mobile_robot.server --config config/pi.local.json
```

On the laptop, first inspect the streams and follower feedback without sending
motion commands:

```bash
python -m mobile_robot.client --config config/laptop.local.json --check
```

`--check` validates the Pi and camera paths. Leader/ESP32 checks happen before
starting the subsequent recording session. Position the leader near the follower
pose to avoid a large initial target change. Then explicitly enable motion:

```bash
python -m mobile_robot.client --config config/laptop.local.json --enable-motion
```

Ctrl+C ends the episode and sends a hold request. Once the devices are closed and
all pending images and telemetry are saved, the laptop asks whether the run was
good or bad. Enter `g` or `b`; Enter alone, Ctrl+C at the prompt, or a noninteractive
terminal leaves it unreviewed. Run the same command again for the next episode;
each directory is unique. `--duration 20` records one timed episode and then asks
the same question.

Each recording starts in **drive to object**. In the recording terminal, press
**Enter** when moving to the next phase:

1. drive to object (initial phase)
2. pick up object
3. drive to the bin
4. drop in bin

The terminal prints the active phase. Enter in phase 4 keeps that phase active;
use Ctrl+C to finish and then classify the run. Phase changes only label data;
they do not pause recording or issue robot actions. Each `telemetry.jsonl` row
contains `phase`, a one-based `phase_index`, and `episode_elapsed_s`. This labels
the arm/controller sample and its three camera references together. Images are
still stored once and can be referenced by rows on either side of a boundary.
`metadata.json` lists `phase_names` and, after saving, `phase_transitions` with
zero-based `start_row`, wall-clock `timestamp`, and monotonic `elapsed_s` for each
phase's first saved row. A stopped run may have fewer than four phases.

Keyboard input is polled without blocking the control loop. Keys queued during
startup/shutdown are discarded so they cannot accidentally advance a phase or
answer the good/bad question. Type Enter alone; text followed by Enter is ignored
during recording. Without an interactive terminal, recording continues in phase
1 and reports that keyboard phase changes are unavailable. Each new episode
starts again at phase 1; old episodes are not relabeled.

Real recordings now default to `training_dataset/` on the laptop:

```text
training_dataset/
  good/episode_.../
  bad/episode_.../
  unreviewed/episode_.../
```

Each episode contains `metadata.json`, `telemetry.jsonl`, and `images/`. Metadata
includes the quality label and recording completion status. Recording begins in
`unreviewed`; classification moves the entire closed episode without changing its
relative image references. These are raw training captures, not yet a converted
LeRobot training dataset. Failed recordings remain unreviewed with error details.
The independently controlled RC car must still be stopped with its controller.

Existing captures under `data/episode_*` remain where they were. Classify an older
closed episode or revise a label with:

```bash
./robot classify data/episode_YOUR_RUN good
./robot classify training_dataset/good/episode_YOUR_RUN bad
```

`--output PATH` overrides the dataset root. `--quality good|bad|unreviewed` skips
the question for automated runs; use `--quality ask` to explicitly enable it.
Mock recordings stay under `data/mock-recordings` by default and never prompt
unless `--quality ask` is supplied. Error-marked episodes cannot be labeled good.

There is no automatic reconnect/replay of motion after a fault.

The control token prevents accidental unauthorized clients but is **not encrypted**.
Camera streams are not authenticated. Use a trusted local network, do not expose
ports 5555/5556 to the Internet, and use a VPN if an untrusted network is necessary.

## Behavior on faults

- Arm control, each camera's capture/encoding, image transport and disk writes
  have separate execution paths. A slow camera cannot block the Pi's command loop.
- The Pi requires a fresh session and one-use command grant, validates all six
  finite joint targets, and rejects old/out-of-order commands.
- After 0.5 seconds without a valid command, the Pi attempts to hold the **measured
  current pose** and invalidates the session. A clean laptop exit requests the
  same hold. Normal Pi shutdown keeps servo holding torque enabled, so support the
  arm before intentionally removing power.
- This is a software watchdog. It cannot guarantee a hold after Pi power loss,
  a frozen process, a disconnected servo bus or a motor fault. It does not stop
  the independently controlled RC car.
- Missing/stale camera frames and stale required ESP32 telemetry stop recording
  and end the motion session. There is no silent substitution of zero/mock data.
- A bounded dataset queue raises an error if storage cannot keep up; it never
  silently discards rows. Network video may drop frames to avoid growing latency.

## Dataset and the steering/throttle correction

`esp32_controller/src/main.cpp` prints `dacSteer`, then `dacThrot`. The old unified
recorder interpreted these in reverse. Both current paths now share the same
parser. This fixes the saved labels, not the physical RC controller wiring or
firmware. The values are raw DAC readings (0..255), **not normalized actions**;
neutral/deadband/direction need measurement on the real remote.

Each episode contains `metadata.json`, `telemetry.jsonl` and JPEGs in `images/`.
Each new wireless row contains:

- `leader_action`: desired leader target;
- `applied_action`: follower target after the configured step clamp;
- `follower_joints`: actual feedback from the follower's position registers;
- `rc_state`: steering/throttle values, sample age and units;
- per-camera frame sequence, file, capture receipt timestamp, clock domain and
  estimated frame age, plus command sequence and command round-trip time.

Multiple control rows can reference the same image when camera FPS is lower than
control FPS. Images are saved once. Timestamps represent software capture receipt,
not synchronized sensor exposure. Pi and laptop wall clocks can differ. Freshness
checks compare monotonic times within each host, related through control replies;
the reported age includes a conservative round-trip allowance. For precise
cross-camera alignment, configure clock synchronization and characterize capture
latency on hardware. This is a custom JSONL/JPEG dataset, not yet a native LeRobot
training dataset; conversion must preserve observation/action semantics.

Existing recordings are not rewritten. If made with the old unified recorder,
their steering/throttle labels may be reversed and `follower_joints` contained
leader targets, not measured follower state. The standalone files in
`pi_onboard/training_logger.py` and `esp32_controller/laptop_utils/` are older
experiments; use the entry points above. In particular, the current ESP firmware
does not read serial drive commands from `test_drive.py`.

## Tests

```bash
python -m pip install -r requirements-test.txt
python -m pytest -q
python -m ruff check mobile_robot tests pi_onboard/pi_follower.py laptop_code/laptop_teleop.py laptop_unified/record_episode.py
```

Tests use real localhost TCP connections, separate simulated Pi/laptop processes,
OpenCV JPEG encoding/decoding, temporary datasets, and fault injection. They cover
command expiry/replay/authentication, lost clients and servers, malformed messages,
camera backpressure/staleness, partial serial lines, recording integrity and
storage errors. A C++ harness executes the actual ESP32 firmware loop with stubbed
ADC/DAC hardware to prove the wire order (`g++` required). LeRobot adapter tests
exercise the installed config classes with mocked USB calls; they skip if LeRobot
is not installed. No test intentionally opens a real USB device.

What remains for the first hardware session: verify the Pi's ARM64 installation,
device paths and camera modes, calibration and joint directions, actual loop/video
latency, watchdog behavior with the supported arm, and operation under motor load
with the final power supply. Software-only tests cannot establish those results.
