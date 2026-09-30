# Episode replay

The prepared episode is `episode_20260927_045310_8df2eb7a` (40.05 seconds of recorded
samples). Watch scene, wrist and car together with one playback timeline:

```bash
./robot replay episode_20260927_045310_8df2eb7a --play
```

The H.264 MP4 is cached under `data/replays/`. Every output frame selects the latest
recorded telemetry row at that time and uses all three image references from that
row. Repeated camera frames are preserved. This aligns the recorded views; it does
not claim the USB cameras were hardware synchronized or actually captured 30 fps.
The video includes phase, elapsed time and original steering/throttle DAC values.
Render another episode with `./robot replay EPISODE_OR_DIRECTORY`; use `--output`
to choose a new MP4 filename. Rendering requires PyAV: `python -m pip install -r requirements-replay.txt`. No PI SDK or API key is needed to render video.

## Robot command replay

This mode replays the recorded **applied arm targets** and **raw steering/throttle
DAC telemetry**, using each row's original timestamp. It does not call a model or
normalize the car signals. It reproduces requested commands, not a guarantee of
the same physical trajectory: vehicle start position, objects, battery, traction,
and calibration still matter.

```bash
./robot replay episode_20260927_045310_8df2eb7a --stream --enable-motion
```

Before starting, use the normal Pi server, flash the updated ESP32 firmware once
with `./robot flash-esp`, confirm the stop neutral values in
`config/inference.local.json`, and place the arm/vehicle at the recorded starting
pose/position with the workspace clear. The code rejects an arm more than five
normalized units from the first target. It does not drive the robot into its
starting pose automatically. No leader arm or PI API key is needed for replay.

The complete command sequence is validated before devices are opened. Gaps over
200 ms, invalid units, absent/noninteger DAC values, and DAC values outside the
current firmware limits are rejected. Replay stops if it falls over 100 ms behind,
if the Pi limits an arm target, or if a command fails. It does not skip samples or
send a catch-up burst. Ctrl+C attempts car neutral and follower hold independently;
the existing device watchdogs remain active.

`--stream --mock` targets a simulated Pi and mock car for testing. A physical replay
has not been run. Render/playback is independent of physical replay; launching the
video does not start actuators.
