# Barebones policy inference

The robot-side runner supports six absolute SO101 joint targets plus steering and
throttle in raw DAC units. It receives follower feedback and two cameras from the
Pi, captures the laptop scene camera, and sends car commands to the laptop's ESP32.
The leader arm is not opened. Ethernet/Wi-Fi selection uses the existing laptop
configuration (`./robot ethernet` / `./robot wifi`).

The PI Fleet adapter now follows the `pi_sdk.inference.PolicyClient` interface in
`/home/czarhc/Downloads/message.txt`. Its configured endpoint is
`wss://api.pi-fleet.com/v1/models/ckpt-1999-rc-arm-pick-ft-sd-v1b`.
Hosted inference runs remotely; no local checkpoint download or Raspberry Pi
reimage is needed. The actual PI SDK, API key, server camera metadata and training
neutral values are still required for a live test. The public `pi-sdk` PyPI
package is a reserved placeholder and cannot be installed.

## PI Fleet setup

`config/inference.pi-fleet.json` is the tracked template. A local copy has been
prepared as `config/inference.local.json`, automatically selected by `./robot infer`.
The local file is ignored by Git. Mock runs continue using the generic config.

1. Obtain the **registry service-account JSON file** supplied by PI during partner
   onboarding. This is separate from the organization API key. Set its absolute
   path as `GOOGLE_APPLICATION_CREDENTIALS=/path/to/pi-sa-key.json` in the ignored
   repository `.env` or shell. The file must contain the actual PI-provided
   service-account credentials; do not create a replacement Google account/key
   and expect access to PI's registry.
2. Create/rotate the **organization API key** in the Partner Portal at
   <https://partner.pi-fleet.com>. It must belong to the organization hosting this
   model. Save `PI_API_KEY=your_key` in `.env` or export it in the shell. A private
   `.env` template has been prepared locally, and `.env.example` documents both
   settings. Keep both credentials out of source control.

   Install the SDK with the prepared command:

   ```bash
   ./robot setup-inference
   ```

   Or provide the registry key path for this invocation:

   ```bash
   ./robot setup-inference --registry-key /absolute/path/to/pi-sa-key.json
   ```

   The command installs the registry authentication helpers, then
   `pi-sdk[video,inference]>=0.3.2` from
   `https://us-east5-python.pkg.dev/pi-external-partners/pi-sdk/simple/`, with PyPI
   as the secondary index, as specified in the provided partner guide. It uses the
   same Python environment as `./robot`, enables the Google Artifact Registry
   keyring backend without interactive password prompts, and preserves the
   network/camera dependency constraints. It does not configure pip globally or
   send robot commands. It checks for a valid credential file before installation.

   `./robot setup-inference --check` reports local readiness without network access,
   installation, or printing credential values. The auth helpers are installed
   locally; private SDK installation still requires the onboarding key. The public
   `pi-sdk` placeholder cannot substitute for the private package.

3. Run `./robot infer --info`. This connects only to the model and prints its
   camera names, action topics and horizon. It opens no USB/cameras/Pi connection
   and does not request an inference. First connection may wait for model startup.
4. Set `camera_map` keys to those **exact server camera names** and values to the
   corresponding training camera roles (`scene`, `wrist`, `car`). Unmapped model
   cameras are errors; the adapter does not insert black frames or guess from
   camera numbers. Use the same views the model saw in training.
5. Set `training_rc_neutral` to `{"steer": ..., "throttle": ...}` from
   `training/splits/pi07_sd_v1.json` or the training episode metadata. Separately
   set the verified integer stop DAC values `neutral_steer` and `neutral_throttle`.
   The adapter rejects differences greater than 15 DAC counts by default, matching
   the supplied script's tolerance. This check does not measure live neutral.
   Check the mapping, calibration and 30 Hz training rate; set `mapping_confirmed`
   only once confirmed. Never copy simulation neutral values onto the car.

The supplied script establishes these model topics:

| Topic | Shape | Meaning |
| --- | --- | --- |
| `observation/arm/joints/position` | (5,) | First five calibrated SO101 positions |
| `observation/arm/gripper/position` | (1,) | Calibrated gripper position |
| `observation/base/drive` | (2,) | (previous DAC − training neutral) / 127.5 |
| `action/arm/joints/position` | (T,5) | Absolute arm positions |
| `action/arm/gripper/position` | (T,1) | Absolute gripper positions |
| `action/base/drive` | (T,2) | Centered steering/throttle offsets |

The adapter converts drive outputs with
`DAC = training_neutral + model_drive * 127.5`, preserving the training coordinate
system. This differs from passing model outputs directly to the DAC. The runner
then checks ranges and rounds to integer DAC values. It does not silently clip
out-of-range predictions to the car's limits.

With the existing Pi server running, first predict without sending commands:

```bash
./robot infer --dry-run --duration 10
```

Dry run reads fresh ESP32 telemetry without requesting a serial reset and reads
normalized follower feedback plus cameras. It never starts an arm control session,
arms the ESP32, or sends DAC commands. It can use the existing telemetry-only ESP
firmware. Do not run it alongside a recording/serial monitor. The normal Pi server
is required because its `--read-only` variant reports raw encoder counts, which do
not match the model's calibrated state units.

For autonomous **car** control, flash the updated ESP32 once, with the car powered
off and serial monitors/recordings closed:

```bash
./robot flash-esp
```

This targets the `esp32_port` in laptop configuration rather than auto-selecting
among the arm/controller USB devices. No firmware is flashed automatically. The
Pi needs its normal server, not an SD card reflash. The old `C,steer,throttle`
commands in the supplied example are replaced by this repository's acknowledged,
session-based ESP protocol with its independent watchdog.

After confirming neutral and the dry run, with the mechanism clear and the wheels
raised for the first drive check:

```bash
./robot infer --enable-motion
```

Ctrl+C ends the session and attempts car neutral/follower hold. Restore joystick
passthrough with `./robot rc-manual` when ready for manual control.

## Hardware-free test, one command

```bash
./robot simulate --infer --duration 5
```

This starts a separate simulated Pi, streams three synthetic cameras over TCP,
and checks mock policy arm + DAC outputs. Logs go under `data/validation/`.
It never opens physical USB devices. It does not benchmark π0.7 inference speed.

## Configure once when the checkpoint arrives

Copy `config/inference.json` to `config/inference.local.json` (ignored by Git).
For a custom runtime, set `adapter`, `checkpoint`, and the verified `neutral_steer` / `neutral_throttle`.
Confirm `state_names`, `action_names`, `camera_map`, `fps`, and `execution_horizon`
against the training pipeline, then set `mapping_confirmed` to true. Defaults are
placeholders, not evidence of the checkpoint's mapping. The local file is selected automatically; override with `--inference-config PATH`.

The model adapter factory is imported as `module_name:function_name` and called:

```python
policy = factory(checkpoint=checkpoint, config=inference_config)
result = policy.infer(observation)
```

The factory loads the actual training runtime and statistics before hardware is
opened. Install that runtime in the environment selected by `./robot`, or provide
a Python adapter which calls your existing inference server. Never have the
adapter open actuators itself. Only one worker invokes `infer`, with one request
in flight. A hung call is abandoned on shutdown; it is not retried.

`observation` contains:

- `state`: float32 array of eight values in `state_names` order. Joint positions
  are measured follower positions, normalized -100..100 (gripper 0..100). Car
  values are the last acknowledged DAC command, initially the configured neutral;
  these are not measurements of wheel position or vehicle speed.
- `prompt`: configured task text.
- Each key in `camera_map`: an RGB uint8 H×W×3 NumPy array, at the actual configured
  camera resolution. The adapter performs the training-specific resize, state/image
  normalization, key conversion, and batching.

Return an array `(8,)` or `(T,8)`, or `{"actions": array}`. The adapter must apply
training statistics to **denormalize** outputs first. Six joint outputs are absolute
positions in the same units above; the remaining outputs are raw steering and
throttle DAC values. Delta actions, radians, padded action dimensions and normalized
network outputs require explicit adapter conversion. The runner rejects invalid
shapes/nonfinite values/out-of-range outputs and rounds valid fractional DAC values.
It validates the entire returned chunk and executes at most `execution_horizon`
rows at `fps` before obtaining a new observation. This minimal runner pauses at
chunk boundaries while inference runs; it has no action-chunk overlap/prefetch.

The current firmware preserves your throttle limit **50..145** and steering
0..255. Out-of-range model predictions stop the run instead of being silently
clipped. Neutral must be measured on your transmitter; 128/100 used in simulation
are not physical defaults. Confirm the existing follower calibration matches
training, particularly if your training exporter used leader targets or other units.

## ESP32 preparation

Build and upload `esp32_controller` using your normal PlatformIO process before
physical inference. The code has not been flashed automatically. Do the upload
and first neutral test with wheels raised and the mechanism clear.

The firmware preserves manual control on boot. It accepts newline-delimited:
`HELLO`, `ARM,session,neutralSteer,neutralThrottle`,
`DRIVE,session,sequence,steer,throttle`, `STOP,session`, and `MANUAL`.
Commands are acknowledged. Invalid ranges, sessions and replayed sequence numbers
are rejected. After 350 ms without a valid drive command, or after STOP, the ESP
latches the specified neutral outputs and rejects late DRIVE messages. The Pi
retains its independent follower watchdog. These are software safeguards, not a
physical emergency stop: power loss/reset, radio behavior and mechanical stopping
distance still depend on hardware. An ESP reset returns to manual control.

## Run

Start the existing Pi control server first. With the adapter, checkpoint,
configuration, calibration and updated firmware ready, on the laptop:

```bash
./robot infer --inference-config config/inference.local.json --enable-motion
```

Optionally override `--checkpoint /path/to/checkpoint`, `--adapter module:factory`,
`--duration 10`, or `--output data/inference/my_run.jsonl` (must be a new file).
Do not run recording or a serial monitor concurrently on the same ESP32.

While awaiting model results the car receives neutral and the arm receives its
measured position as a hold target. Inference timeout, stale cameras, invalid
outputs, transport errors, or Ctrl+C end the run and independently attempt car
neutral and follower hold. No network reconnection or old-action replay occurs.
Repeated neutral pauses may affect policies trained on continuous driving; measure
real model latency before increasing the executed chunk length.

Action logs save requested arm targets, car DAC values, timestamps and whether the
loop was predicting or waiting. These are diagnostic logs, not training episodes
or proof the robot physically reached each target.

After autonomous inference, restore handheld passthrough explicitly:

```bash
./robot rc-manual
```

This restores live joystick control; center the transmitter controls first.

## Verified hosted-model integration

PI SDK 0.3.2 and its inference/video dependencies are installed on the laptop.
The supplied registry credential authenticated successfully; both credentials are
kept in ignored local files. Dependency checks pass. The configured hosted endpoint
responded to metadata requests, and its saved training profile was read directly.
It selects scene + wrist images (not car), and absolute joint/gripper/drive topics
in the adapter's order. This server omits `action_keys` metadata but declares the
same topics in `output_spec`; the adapter validates that form too.

The local/PI Fleet configs now use the exact server camera topic names and training
task. `phase_control` is enabled: Enter advances through the four recorded phase
labels, sending the current label as `raw_text`. The full task remains separate as
`robot_task_string`. Queued predictions from the previous phase are discarded.

One explicitly approved inference used the first scene/wrist frames and six arm
readings from `episode_20260927_035136_0ca929e8`, with an explicitly zero model drive
state as in the supplied example. It returned ten actions in 2.07 seconds. The
first drive output was approximately `[0.021535, -0.004458]`. Under the supplied
training script's conversion this is a DAC offset `[+2.746, -0.568]`, to be added
to the **actual training neutral**. These outputs must not be sent directly as raw
DAC counts, and their magnitude alone does not prove the training transform.

The exact training neutral and preprocessing source are still required; they were
not present in the hosted training profile or the inspected episode annotations.
`mapping_confirmed` and neutral fields remain unset until verified. Physical
inference has not run. The two-second measured request time also means the basic
10-action/30-Hz runner pauses at chunk boundaries; this is not continuous 30-Hz
model inference or a claim of smooth autonomous driving.

Checks cover the actual SDK's H.264/MessagePack path, distinct steering/throttle
values, 256 valid DAC round trips, and compiled firmware serial parsing through
DAC pin writes with neutral timeout. These are software tests, not measured output
voltages or proof of wheel behavior. See `REPLAY.md` for video and command replay.
