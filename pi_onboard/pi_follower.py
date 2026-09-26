import cv2
import zmq
import time
import json
import os
import numpy as np

# --- 1. LEROBOT IMPORTS ---
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

# --- CONFIGURATION ---
# Linux usually assigns even numbers to physical cameras (0, 2, 4). 
# If they fail to open, try [0, 1, 2] instead.
CAM_INDICES = [0, 2, 4] 
DATASET_DIR = "training_dataset"
FOLLOWER_PORT = "/dev/ttyACM0" 

def set_chassis_velocity(rc_state):
    # Placeholder: Pass throttle/steer to RC car motors via Serial/I2C/PWM
    pass

def main():
    # 1. Initialize 3 Cameras
    caps = [cv2.VideoCapture(idx) for idx in CAM_INDICES]
    for i, cap in enumerate(caps):
        if not cap.isOpened():
            print(f"Warning: Camera {CAM_INDICES[i]} not found!")

    # 2. Initialize ZeroMQ Server
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    # 3. Initialize LeRobot Follower Arm
    print("Connecting to Follower arm...")
    robot_cfg = SO100FollowerConfig(port=FOLLOWER_PORT)
    follower = SO100Follower(robot_cfg)
    follower.connect()
    print("Follower arm connected! Waiting for laptop commands...")
    
    current_episode = None
    log_file = None
    frame_idx = 0
    
    try:
        while True:
            msg = socket.recv_json()
            cmd = msg.get("command")
            
            # --- APPLY PHYSICAL MOVEMENTS INSTANTLY ---
            if "leader_joints" in msg:
                action = msg["leader_joints"]
                if isinstance(action, list):
                    action = np.array(action, dtype=np.float32)
                follower.send_action(action)
                
            if "rc_state" in msg:
                set_chassis_velocity(msg["rc_state"])

            # --- RECORDING LOGIC ---
            if cmd == "record":
                ep_name = msg.get("episode")
                
                # Setup new episode folder
                if current_episode != ep_name:
                    current_episode = ep_name
                    frame_idx = 0
                    ep_dir = f"{DATASET_DIR}/{ep_name}"
                    os.makedirs(f"{ep_dir}/images", exist_ok=True)
                    log_file = open(f"{ep_dir}/telemetry.jsonl", "a")
                    print(f"Started recording: {ep_name}")

                # Read all 3 cameras instantly
                frames = [cap.read()[1] for cap in caps]
                timestamp = time.time()
                
                saved_images = {}
                for cam_id, frame in enumerate(frames):
                    if frame is not None:
                        img_name = f"frame_{frame_idx:04d}_cam{cam_id}.jpg"
                        cv2.imwrite(f"{DATASET_DIR}/{current_episode}/images/{img_name}", frame)
                        saved_images[f"cam_{cam_id}"] = img_name
                
                # Save the single synced row
                log_entry = {
                    "timestamp": timestamp,
                    "image_files": saved_images,
                    "follower_joints": msg.get("leader_joints", []),
                    "rc_state": msg.get("rc_state", {})
                }
                log_file.write(json.dumps(log_entry) + "\n")
                frame_idx += 1
                
                socket.send_json({"status": "recorded"})

            # --- STOP RECORDING LOGIC ---
            elif cmd == "stop_recording":
                if log_file and not log_file.closed:
                    log_file.close()
                    print(f"Saved {frame_idx} frames. Episode complete.")
                current_episode = None
                socket.send_json({"status": "file_closed"})
                
            # --- IDLE LOGIC ---
            elif cmd == "idle":
                socket.send_json({"status": "zero_velocity_applied"})

    finally:
        # Safe Hardware Cleanup
        for cap in caps:
            cap.release()
        follower.disconnect() 
        if log_file and not log_file.closed:
            log_file.close()
        print("Pi hardware released cleanly.")

if __name__ == "__main__":
    main()