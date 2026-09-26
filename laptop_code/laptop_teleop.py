import zmq
import serial
import time
import json

PI_IP_ADDRESS = "10.42.0.181" # The Ethernet IP
ESP32_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def get_leader_joints():
    # Placeholder: Read your Leader arm hardware here
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    print(f"Connecting to Pi at {PI_IP_ADDRESS}...")
    socket.connect(f"tcp://{PI_IP_ADDRESS}:5555")
    
    try:
        esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
        print("ESP32 connected.")
    except Exception:
        esp32 = None
        print("Warning: ESP32 not found. Running mock RC commands.")

    episode_count = 0

    while True:
        ep_name = f"episode_{episode_count:03d}"
        
        # 1. IDLE STATE
        try:
            input(f"\n[IDLE] Press ENTER to start recording {ep_name} (or double Ctrl+C to quit)...")
        except KeyboardInterrupt:
            print("\nExiting program.")
            break

        print(f"\n[RECORDING] {ep_name} is active! Drive the robot.")
        print("Press Ctrl+C to STOP recording this episode.")
        
        # 2. ACTIVE STATE
        try:
            while True:
                # Read ESP32 steering
                rc_state = {"throttle": 0.0, "steer": 0.0}
                if esp32 and esp32.in_waiting > 0:
                    try:
                        line = esp32.readline().decode('utf-8').strip()
                        rc_state = json.loads(line)
                    except:
                        pass
                
                # Send frame data to Pi
                payload = {
                    "command": "record",
                    "episode": ep_name,
                    "rc_state": rc_state,
                    "leader_joints": get_leader_joints()
                }
                
                socket.send_json(payload)
                socket.recv_json() # Wait for Pi to finish saving
                
                time.sleep(0.033) # Run at ~30Hz

        # 3. STOP STATE
        except KeyboardInterrupt:
            print(f"\n[STOPPED] Ending {ep_name}. Closing file on Pi...")
            
            # Step A: Stop recording cleanly
            socket.send_json({"command": "stop_recording"})
            socket.recv_json()
            
            print("File safely saved. Zeroing out robot velocity...")
            
            # Step B: Apply hard brake to RC chassis and hold arm
            socket.send_json({
                "command": "idle",
                "rc_state": {"throttle": 0.0, "steer": 0.0},
                "leader_joints": get_leader_joints()
            })
            socket.recv_json()
            
            episode_count += 1

if __name__ == "__main__":
    main()