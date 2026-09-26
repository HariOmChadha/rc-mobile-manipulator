import cv2
import zmq
import time
import json
import os
import numpy as np

# --- 1. IMPORT LEROBOT FOLLOWER ---
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

CAM_INDEX = 0
DATASET_DIR = "training_dataset"

# Set this to the USB port the Follower arm is plugged into on the Pi
FOLLOWER_PORT = "/dev/ttyACM0" 

def set_chassis_velocity(rc_state):
    # Pass throttle/steer to RC car motors
    pass

def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    # --- 2. CONNECT LEROBOT FOLLOWER ---
    print("Connecting to Follower arm...")
    robot_cfg = SO100FollowerConfig(port=FOLLOWER_PORT)
    follower = SO100Follower(robot_cfg)
    follower.connect()
    print("Follower arm connected and calibrated!")
    
    current_episode = None
    log_file = None
    frame_idx = 0
    
    try:
        while True:
            msg = socket.recv_json()
            cmd = msg.get("command")
            
            # --- 3. APPLY PHYSICAL MOVEMENTS ---
            if "leader_joints" in msg:
                # Convert the incoming Python list/dict back into the format LeRobot expects
                action = msg["leader_joints"]
                if isinstance(action, list):
                    action = np.array(action, dtype=np.float32)
                
                # Send the joint angles directly to the motors!
                follower.send_action(action)
                
            if "rc_state" in msg:
                set_chassis_velocity(msg["rc_state"])

            if cmd == "record":
                ep_name = msg.get("episode")
                if current_episode != ep_name:
                    current_episode = ep_name
                    frame_idx = 0
                    ep_dir = f"{DATASET_DIR}/{ep_name}"
                    os.makedirs(f"{ep_dir}/images", exist_ok=True)
                    log_file = open(f"{ep_dir}/telemetry.jsonl", "a")

                ret, frame = cap.read()
                timestamp = time.time()
                
                if ret:
                    img_name = f"frame_{frame_idx:04d}.jpg"
                    cv2.imwrite(f"{DATASET_DIR}/{current_episode}/images/{img_name}", frame)
                    
                    log_entry = {
                        "timestamp": timestamp,
                        "image_file": img_name,
                        "follower_joints": msg.get("leader_joints", []),
                        "rc_state": msg.get("rc_state", {})
                    }
                    log_file.write(json.dumps(log_entry) + "\n")
                    frame_idx += 1
                    
                socket.send_json({"status": "recorded"})

            elif cmd == "stop_recording":
                if log_file and not log_file.closed:
                    log_file.close()
                current_episode = None
                socket.send_json({"status": "file_closed"})
                
            elif cmd == "idle":
                socket.send_json({"status": "zero_velocity_applied"})

    finally:
        cap.release()
        follower.disconnect() # Safely release the arm motors
        if log_file and not log_file.closed:
            log_file.close()

if __name__ == "__main__":
    main()