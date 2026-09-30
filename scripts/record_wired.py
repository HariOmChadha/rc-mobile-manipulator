"""Wired fallback recorder. Network operation lives in mobile_robot.client."""

import json
import sys
import time
import uuid
from contextlib import ExitStack
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_robot.hardware import RCReader  # noqa: E402

LEADER_PORT = "/dev/ttyACM1"
FOLLOWER_PORT = "/dev/ttyACM0"
ESP32_PORT = "/dev/ttyUSB0"
CAM_INDICES = [2, 4]
DATASET_DIR = "training_dataset"


def positions(values):
    return {key: float(value) for key, value in values.items() if key.endswith(".pos")}


def record_frame(leader, follower, esp32, caps, ep_dir, frame_idx):
    rc_sample = esp32.read()
    if rc_sample["age_s"] is None or rc_sample["age_s"] > 0.5:
        raise RuntimeError("ESP32 telemetry is missing or stale")
    action = positions(leader.get_action())
    applied = positions(follower.send_action(action))
    measured = positions(follower.get_observation())
    state_timestamp = time.time()
    saved_images, image_timestamps = {}, {}
    for cam_id, cap in enumerate(caps):
        ok, frame = cap.read()
        captured = time.time()
        if not ok or frame is None:
            raise RuntimeError(f"Camera {cam_id} stopped returning frames")
        image_name = f"frame_{frame_idx:06d}_cam{cam_id}.jpg"
        if not cv2.imwrite(str(ep_dir / "images" / image_name), frame):
            raise OSError(f"Could not save {image_name}")
        saved_images[f"cam_{cam_id}"] = image_name
        image_timestamps[f"cam_{cam_id}"] = captured
    return {
        "timestamp": state_timestamp,
        "image_files": saved_images,
        "image_timestamps": image_timestamps,
        "leader_action": action,
        "applied_action": applied,
        "follower_joints": measured,
        "rc_state": rc_sample["values"],
        "rc_age_s": rc_sample["age_s"],
        "rc_units": rc_sample["units"],
        "joint_units": "normalized_minus100_100_gripper_0_100",
    }


def disconnect(device):
    if device.bus.is_connected:
        device.disconnect()


def main():
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    print("Initializing wired recording pipeline...")
    with ExitStack() as resources:
        caps = []
        for idx in CAM_INDICES:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            resources.callback(cap.release)
            if not cap.isOpened():
                raise RuntimeError(f"Camera {idx} not found")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            caps.append(cap)
        esp32 = RCReader(ESP32_PORT)
        resources.callback(esp32.close)
        leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT, id="my_leader_arm", use_degrees=False))
        resources.callback(disconnect, leader)
        leader.connect()
        follower = SO101Follower(
            SO101FollowerConfig(
                port=FOLLOWER_PORT,
                id="my_follower_arm",
                use_degrees=False,
                disable_torque_on_disconnect=False,
            )
        )
        resources.callback(disconnect, follower)
        follower.connect()
        while True:
            try:
                input("Press ENTER to record (Ctrl+C to quit)...")
            except KeyboardInterrupt:
                break
            ep_dir = Path(DATASET_DIR) / f"episode_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
            (ep_dir / "images").mkdir(parents=True, exist_ok=False)
            print(f"Recording {ep_dir}. Ctrl+C ends this episode.")
            try:
                with (ep_dir / "telemetry.jsonl").open("x") as log_file:
                    frame_idx = 0
                    while True:
                        start = time.monotonic()
                        row = record_frame(leader, follower, esp32, caps, ep_dir, frame_idx)
                        log_file.write(json.dumps(row, allow_nan=False) + "\n")
                        log_file.flush()
                        frame_idx += 1
                        time.sleep(max(0, 1 / 30 - (time.monotonic() - start)))
            except KeyboardInterrupt:
                print("Episode saved.")
            finally:
                # Keep holding torque; do not drop an arm carrying a load.
                follower.send_action(positions(follower.get_observation()))


if __name__ == "__main__":
    main()
