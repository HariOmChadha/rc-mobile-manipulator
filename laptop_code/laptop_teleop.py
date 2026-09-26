import zmq
import serial
import time
import json
import numpy as np
from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

PI_IP_ADDRESS = "10.42.0.181"
ESP32_PORT = '/dev/ttyUSB0'

# Local USB for Leader, Virtual socat port for Follower
LEADER_PORT = "/dev/ttyACM0" 
FOLLOWER_PORT = "/tmp/virtual_follower" 

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{PI_IP_ADDRESS}:5555")
    
    print("Connecting to Leader arm...")
    leader = SO100Leader(SO100LeaderConfig(port=LEADER_PORT))
    leader.connect()
    
    print("Connecting to Follower arm (via Virtual Ethernet Port)...")
    follower = SO100Follower(SO100FollowerConfig(port=FOLLOWER_PORT))
    follower.connect()
    
    try: esp32 = serial.Serial(ESP32_PORT, 115200, timeout=0.1)
    except: esp32 = None

    episode_count = 0
    print("\nSystem Ready! All AI compute is running locally.")

    while True:
        ep_name = f"episode_{episode_count:03d}"
        
        try: input(f"\nPress ENTER to start recording {ep_name}...")
        except KeyboardInterrupt: break
        
        try:
            while True:
                # 1. Read ESP32
                rc_state = {"throttle": 0.0, "steer": 0.0}
                if esp32 and esp32.in_waiting > 0:
                    try: rc_state = json.loads(esp32.readline().decode('utf-8').strip())
                    except: pass
                
                # 2. Read Leader & Move Follower (Running locally on Laptop CPU!)
                action = leader.get_action()
                follower.send_action(action)
                
                # Format array for JSON
                joints_list = action.tolist() if hasattr(action, "tolist") else action

                # 3. Tell Pi to snap photos of this exact state
                socket.send_json({
                    "command": "record",
                    "episode": ep_name,
                    "rc_state": rc_state,
                    "leader_joints": joints_list
                })
                socket.recv_json() 
                time.sleep(0.033)

        except KeyboardInterrupt:
            socket.send_json({"command": "stop_recording"})
            socket.recv_json()
            episode_count += 1

if __name__ == "__main__":
    main()