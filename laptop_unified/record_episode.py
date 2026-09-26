import cv2
import serial
import time
import json
import os
import numpy as np

# --- LEROBOT IMPORTS ---
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# --- CONFIGURATION ---
LEADER_PORT = "/dev/ttyACM1"
FOLLOWER_PORT = "/dev/ttyACM0"
ESP32_PORT = "/dev/ttyUSB0"
CAM_INDICES = [2, 4]        # 2 cameras as requested
DATASET_DIR = "training_dataset"

def main():
    print("Initializing Unified Pipeline...")
    
    # 1. Initialize Cameras (Robust V4L2 + Bandwidth Fix)
    caps = []
    for idx in CAM_INDICES:
        # Force the Linux V4L2 backend
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        
        # Drop resolution to 640x480 to prevent USB bus blackout
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not cap.isOpened():
            print(f"Warning: Camera {idx} not found!")
        caps.append(cap)

    print("Warming up cameras and clearing buffers...")
    # Read 30 frames rapidly to let auto-exposure settle
    for _ in range(30):
        for cap in caps:
            cap.read()

    # 2. Initialize ESP32
    try:
        esp32 = serial.Serial(ESP32_PORT, 115200, timeout=0.1)
        print("ESP32 connected.")
    except Exception:
        esp32 = None
        print("Warning: ESP32 not found. Mocking RC commands.")

    # 3. Initialize LeRobot Arms
    print("Connecting Leader...")
    leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT, id="my_leader_arm"))
    leader.connect()
    
    print("Connecting Follower...")
    follower = SO101Follower(SO101FollowerConfig(port=FOLLOWER_PORT, id="my_follower_arm"))
    follower.connect()

    episode_count = 0
    print("\nHardware ready! All compute running locally.")

    try:
        while True:
            ep_name = f"episode_{episode_count:03d}"
            
            # --- IDLE STATE ---
            try:
                input(f"\n[IDLE] Press ENTER to start recording {ep_name} (or double Ctrl+C to quit)...")
            except KeyboardInterrupt:
                break

            print(f"\n[RECORDING] {ep_name} is active! Drive the robot.")
            
            # Setup Episode Folder
            ep_dir = f"{DATASET_DIR}/{ep_name}"
            os.makedirs(f"{ep_dir}/images", exist_ok=True)
            log_file = open(f"{ep_dir}/telemetry.jsonl", "a")
            frame_idx = 0

            # --- ACTIVE STATE ---
            try:
                # Initialize rc_state OUTSIDE the loop so it remembers the last command
                rc_state = {"throttle": 0.0, "steer": 0.0}
                
                while True:
                    loop_start = time.time()
                    
                    # A. Read ESP32 (Zero-Latency Buffer Flush)
                    if esp32 and esp32.in_waiting > 0:
                        try:
                            # Read lines lightning-fast until the buffer is empty
                            latest_line = ""
                            while esp32.in_waiting > 0:
                                latest_line = esp32.readline().decode('utf-8').strip()
                            
                            # Update state ONLY if we got a valid new line
                            if latest_line:
                                parts = latest_line.split(',')
                                if len(parts) == 2:
                                    rc_state["throttle"] = float(parts[0])
                                    rc_state["steer"] = float(parts[1])
                        except Exception:
                            pass
                                                
                    # B. Read Leader & Move Follower
                    action = leader.get_action()
                    follower.send_action(action)
                    joints_list = action.tolist() if hasattr(action, "tolist") else action

                    # C. Snap Photos
                    frames = [cap.read()[1] for cap in caps]
                    timestamp = time.time()
                    
                    saved_images = {}
                    for cam_id, frame in enumerate(frames):
                        if frame is not None:
                            img_name = f"frame_{frame_idx:04d}_cam{cam_id}.jpg"
                            cv2.imwrite(f"{ep_dir}/images/{img_name}", frame)
                            saved_images[f"cam_{cam_id}"] = img_name

                    # D. Log Synced Row
                    log_entry = {
                        "timestamp": timestamp,
                        "image_files": saved_images,
                        "follower_joints": joints_list,
                        "rc_state": rc_state
                    }
                    log_file.write(json.dumps(log_entry) + "\n")
                    frame_idx += 1
                    
                    # Enforce ~30Hz loop rate (33ms)
                    elapsed = time.time() - loop_start
                    time.sleep(max(0, 0.033 - elapsed))

            # --- STOP STATE ---
            except KeyboardInterrupt:
                print(f"\n[STOPPED] Ending {ep_name}. Saving files...")
                if log_file and not log_file.closed:
                    log_file.close()
                episode_count += 1

    finally:
        # Safe Cleanup
        print("\nReleasing hardware...")
        for cap in caps: cap.release()
        leader.disconnect()
        follower.disconnect()
        if esp32: esp32.close()

if __name__ == "__main__":
    main()