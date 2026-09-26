import cv2
import zmq
import time
import json
import os

CAM_INDICES = [0, 2, 4] 
DATASET_DIR = "training_dataset"

def main():
    caps = [cv2.VideoCapture(idx) for idx in CAM_INDICES]
    
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    
    print("Lightweight Pi Server ready! Waiting for ZMQ commands...")
    
    current_episode = None
    log_file = None
    frame_idx = 0
    
    try:
        while True:
            msg = socket.recv_json()
            cmd = msg.get("command")

            if cmd == "record":
                ep_name = msg.get("episode")
                
                if current_episode != ep_name:
                    current_episode = ep_name
                    frame_idx = 0
                    ep_dir = f"{DATASET_DIR}/{ep_name}"
                    os.makedirs(f"{ep_dir}/images", exist_ok=True)
                    log_file = open(f"{ep_dir}/telemetry.jsonl", "a")

                # Snap 3 photos perfectly in sync with the laptop's command
                frames = [cap.read()[1] for cap in caps]
                timestamp = time.time()
                
                saved_images = {}
                for cam_id, frame in enumerate(frames):
                    if frame is not None:
                        img_name = f"frame_{frame_idx:04d}_cam{cam_id}.jpg"
                        cv2.imwrite(f"{DATASET_DIR}/{current_episode}/images/{img_name}", frame)
                        saved_images[f"cam_{cam_id}"] = img_name
                
                # Log the state that the laptop calculated
                log_entry = {
                    "timestamp": timestamp,
                    "image_files": saved_images,
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
                socket.send_json({"status": "zeroed"})

    finally:
        for cap in caps: cap.release()
        if log_file and not log_file.closed: log_file.close()

if __name__ == "__main__":
    main()