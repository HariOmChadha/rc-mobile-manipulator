import cv2
import zmq
import time
import json
import os

CAM_INDEX = 0
DATASET_DIR = "training_dataset"

def set_follower_joints(joints):
    # Placeholder: Pass angles to Follower arm motors
    pass

def set_chassis_velocity(rc_state):
    # Placeholder: Pass throttle/steer to RC car motors via Serial/I2C/PWM
    pass

def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Warning: Camera not found! Check USB connection.")
        
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    print("Pi Follower ready. Waiting for laptop commands...")
    
    current_episode = None
    log_file = None
    frame_idx = 0
    
    try:
        while True:
            msg = socket.recv_json()
            cmd = msg.get("command")
            
            # 1. Always apply physical movements instantly
            if "leader_joints" in msg:
                set_follower_joints(msg["leader_joints"])
            if "rc_state" in msg:
                set_chassis_velocity(msg["rc_state"])

            # 2. Handle Recording state
            if cmd == "record":
                ep_name = msg.get("episode")
                
                # Open new folder if starting an episode
                if current_episode != ep_name:
                    current_episode = ep_name
                    frame_idx = 0
                    ep_dir = f"{DATASET_DIR}/{ep_name}"
                    os.makedirs(f"{ep_dir}/images", exist_ok=True)
                    log_file = open(f"{ep_dir}/telemetry.jsonl", "a")
                    print(f"Started recording: {ep_name}")

                # Snap and save photo
                ret, frame = cap.read()
                timestamp = time.time()
                
                if ret:
                    img_name = f"frame_{frame_idx:04d}.jpg"
                    cv2.imwrite(f"{DATASET_DIR}/{current_episode}/images/{img_name}", frame)
                    
                    # Log state
                    log_entry = {
                        "timestamp": timestamp,
                        "image_file": img_name,
                        "follower_joints": msg.get("leader_joints", []),
                        "rc_state": msg.get("rc_state", {})
                    }
                    log_file.write(json.dumps(log_entry) + "\n")
                    frame_idx += 1
                    
                socket.send_json({"status": "recorded"})

            # 3. Handle Stop Recording state
            elif cmd == "stop_recording":
                if log_file and not log_file.closed:
                    log_file.close()
                    print(f"Saved {frame_idx} frames. Episode complete.")
                current_episode = None
                socket.send_json({"status": "file_closed"})
                
            # 4. Handle Idle state (Motors were already zeroed in step 1)
            elif cmd == "idle":
                socket.send_json({"status": "zero_velocity_applied"})

    finally:
        cap.release()
        if log_file and not log_file.closed:
            log_file.close()
        print("Pi hardware released cleanly.")

if __name__ == "__main__":
    main()