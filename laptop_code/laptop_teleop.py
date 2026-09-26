import zmq
import serial
import time
import json
import numpy as np

# --- 1. IMPORT LEROBOT LEADER ---
from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig

PI_IP_ADDRESS = "10.42.0.181"
ESP32_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Set this to the USB port your Leader arm is plugged into on the laptop
LEADER_PORT = "/dev/ttyACM0" 

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{PI_IP_ADDRESS}:5555")
    
    # --- 2. CONNECT LEROBOT LEADER ---
    print("Connecting to Leader arm...")
    leader_cfg = SO100LeaderConfig(port=LEADER_PORT)
    leader = SO100Leader(leader_cfg)
    leader.connect()
    print("Leader arm connected and calibrated!")
    
    try:
        esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
    except Exception:
        esp32 = None

    episode_count = 0

    while True:
        ep_name = f"episode_{episode_count:03d}"
        
        try:
            input(f"\n[IDLE] Press ENTER to start recording {ep_name} (or double Ctrl+C to quit)...")
        except KeyboardInterrupt:
            break

        print(f"\n[RECORDING] {ep_name} is active!")
        
        try:
            while True:
                rc_state = {"throttle": 0.0, "steer": 0.0}
                if esp32 and esp32.in_waiting > 0:
                    try:
                        rc_state = json.loads(esp32.readline().decode('utf-8').strip())
                    except:
                        pass
                
                # --- 3. READ LEADER JOINTS ---
                action = leader.get_action()
                
                # LeRobot returns tensors/numpy arrays. Convert to a standard list so it can be sent via JSON.
                if hasattr(action, "tolist"):
                    leader_joints = action.tolist()
                elif isinstance(action, dict):
                    leader_joints = {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in action.items()}
                else:
                    leader_joints = action

                payload = {
                    "command": "record",
                    "episode": ep_name,
                    "rc_state": rc_state,
                    "leader_joints": leader_joints
                }
                
                socket.send_json(payload)
                socket.recv_json() 
                time.sleep(0.033)

        except KeyboardInterrupt:
            print(f"\n[STOPPED] Ending {ep_name}. Closing file on Pi...")
            socket.send_json({"command": "stop_recording"})
            socket.recv_json()
            
            # Send zero velocity. (We don't send arm coordinates here to let it rest)
            socket.send_json({
                "command": "idle",
                "rc_state": {"throttle": 0.0, "steer": 0.0}
            })
            socket.recv_json()
            episode_count += 1

if __name__ == "__main__":
    main()