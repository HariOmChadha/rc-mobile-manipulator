# Swapping and comparing cameras

Camera slots describe mounting locations: `wrist` and `car` belong to the Pi,
and `scene` belongs to the laptop. Profiles describe camera models and requested
capture modes. You can change any one slot without changing the arm code or the
other cameras. Stop the host process before unplugging cameras or applying a new
configuration; automatic hot swapping during a recording is not implemented.

## INNO-MAKER U20CAM-720P

The [manufacturer's specification](https://www.inno-maker.com/product/u20cam-720p/)
lists a UVC USB camera with a 120° diagonal / 102° horizontal field of view and
MJPEG modes of 1280×720, 800×600, 640×480 and 320×240 at 30 fps. The supplied
profiles use its documented 720p and VGA MJPEG modes. This is a software preset,
not a certification that the device has been tested on your Pi.

The current cameras' field of view is unknown, so we cannot yet quantify an
improvement. Lens FOV is not increased by a software setting. A 4:3 capture mode
can have different cropping from 16:9; don't assume VGA retains the 720p view.
Wider views also spread a small object over fewer pixels at the same resolution.
Compare framing and gripper/object detail from the same mounting position.

## Profiles and individual slots

```bash
./robot cameras list
./robot cameras show --config config/pi.local.json
```

On the Pi, select a physical camera by its stable device path:

```bash
./robot cameras set --config config/pi.local.json \
  --camera wrist --profile innomaker-u20cam-720p \
  --device /dev/v4l/by-id/YOUR_CAMERA-video-index0 --label wrist-wide
```

Use `--camera car` for the other onboard camera. For the scene camera, run the
same command on the laptop with `--config config/laptop.local.json --camera scene`.
Use `./robot devices` to discover paths. Identical cameras may lack unique serial
numbers; `/dev/v4l/by-path/` then binds the slot to a particular USB port. Do not
identify a camera solely by the order `/dev/video0`, `/dev/video2`, etc. appear.

Switch back with `--profile generic-vga`. Override individual settings with
`--width`, `--height`, `--fps`, `--fourcc` or `--jpeg-quality`. Overrides are
requests, not claims that a device supports them. Lower JPEG quality changes the
encoded stream; it does not widen the optics or reduce the camera's USB capture
rate. Add other models in `config/camera_profiles.json` using the same schema.

INNO-MAKER profiles enable `strict_mode`: reported mode mismatches stop capture
with an explanation instead of silently recording another resolution/FPS/format.
Some drivers do not report all settings; unknown fields are logged and saved as
null, and actual frame dimensions are still checked. Inspect measured FPS in a
longer probe to assess the achieved capture rate. `--no-strict-mode` allows a
deliberate fallback while preserving the requested and reported values.

## Save separate experiments

Use `--output` to create a variant without modifying the active configuration:

```bash
./robot cameras set --config config/pi.local.json --camera wrist \
  --profile innomaker-u20cam-720p --output config/pi-wide.local.json
./robot pi --config config/pi-wide.local.json
```

Three ready-made templates are included:

| File | Change |
| --- | --- |
| `config/experiments/pi-wrist-wide.json` | Wrist at 1280×720/30 MJPEG |
| `config/experiments/pi-wrist-wide-vga.json` | Wrist at 640×480/30 MJPEG |
| `config/experiments/laptop-scene-wide.json` | Scene at 1280×720/30 MJPEG |

These templates have example USB paths and laptop loopback networking. For real
use, create variants from your configured `*.local.json` files as shown above.
Relative calibration paths are rebased when saving to a different directory.
When applying a Pi profile from this laptop, rebuild/transfer the bundle or copy
the changed config and profile catalog to the Pi. Edits here don't change a
running remote machine.

## Compare without moving the arm

Run this **on the host to which the cameras are connected**, with the normal
server/client stopped so two processes do not compete for the same USB device:

```bash
./robot cameras probe --config config/pi-wide.local.json --duration 5 --label wrist-wide
```

The probe opens cameras only, never the arm or ESP32. It saves a sample JPEG for
each camera and a JSON report with requested/reported settings, observed capture
FPS, JPEG size and estimated JPEG payload bandwidth under `data/camera-tests/`.
It tests all cameras in that config together. The bandwidth estimate excludes
network overhead, and the probe doesn't measure optical FOV or Wi-Fi performance.
Use the same scene, mount, lighting and test duration for comparisons.

Without hardware, exercise the preset and image pipeline using:

```bash
./robot cameras probe --config config/experiments/pi-wrist-wide.json --mock --duration 3
./robot simulate --pi-config config/experiments/pi-wrist-wide.json --duration 5
```

The simulation handles mixed camera resolutions. It does not simulate wide-angle
optics, distortion or exposure. For a physical trial, start the chosen Pi config,
then on the laptop:

```bash
./robot check
./robot record --enable-motion --duration 5 --label wrist-wide-720p
```

Every frame's metadata stores its slot, profile/model, device, mounting label,
manufacturer FOV claim/source, requested mode, reported mode and actual dimensions.
Episode metadata saves the experiment label. The returned Pi frame metadata
travels with the images, so the laptop also records remote camera identity.
Changing resolution changes dataset image shapes; changing lens/mount changes
the observation distribution. Keep experiments separate until a deliberate
training-data conversion or retraining step accounts for those differences.
