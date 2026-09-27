# Barebones policy inference

The robot-side runner supports six absolute SO101 joint targets plus steering and
throttle in raw DAC units. It receives follower feedback and two cameras from the
Pi, captures the laptop scene camera, and sends car commands to the laptop's ESP32.
The leader arm is not opened. Ethernet/Wi-Fi selection uses the existing laptop
configuration (`./robot ethernet` / `./robot wifi`).

**π0.7 checkpoint loading is not wired yet.** We need the training repository or
its inference API. A model name alone does not identify the weight format,
normalization, action order, camera names, or runtime. No real checkpoint or
physical autonomous movement has been tested. This runner accepts a training-specific
Python adapter; it does not relabel a π0/π0.5 loader as π0.7.

## Hardware-free test, one command

```bash
./robot simulate --infer --duration 5
```

This starts a separate simulated Pi, streams three synthetic cameras over TCP,
and checks mock policy arm + DAC outputs. Logs go under `data/validation/`.
It never opens physical USB devices. It does not benchmark π0.7 inference speed.

## Configure once when the checkpoint arrives

Copy `config/inference.json` to `config/inference.local.json` (ignored by Git).
Set `adapter`, `checkpoint`, and the verified `neutral_steer` / `neutral_throttle`.
Confirm `state_names`, `action_names`, `camera_map`, `fps`, and `execution_horizon`
against the training pipeline, then set `mapping_confirmed` to true. Defaults are
placeholders, not evidence of the checkpoint's mapping. Select the local file with
`--inference-config config/inference.local.json`.

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
