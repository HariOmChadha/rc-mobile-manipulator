import cv2
import serial
import time
import json
import os

# Configuration - Update these to match your Pi's hardware addresses later
CAM_INDEX = 0
ESP32_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
OUTPUT_DIR = "training_dataset"

def main():
    print("Setting up Training Logs Mode...")
    os.makedirs(f"{OUTPUT_DIR}/images", exist_ok=True)
    log_file = open(f"{OUTPUT_DIR}/telemetry.jsonl", "a")

    # 1. Initialize Hardware
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    try:
        esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
    except serial.SerialException:
        print(f"Warning: ESP32 not found on {ESP32_PORT}. Running in mock mode.")
        esp32 = None

    print("Hardware initialized. Collecting data at ~30Hz. Press Ctrl+C to stop.")
    
    frame_count = 0
    try:
        while True:
            # 2. Time Synchronization - Grab exact timestamp at loop start
            timestamp = time.time()
            
            # 3. Read Camera
            ret, frame = cap.read()
            if not ret:
                continue
            
            # 4. Read ESP32 (Chassis/Drive Telemetry)
            chassis_state = {}
            if esp32 and esp32.in_waiting > 0:
                try:
                    # Assuming your ESP32 streams state as JSON strings
                    line = esp32.readline().decode('utf-8').strip()
                    chassis_state = json.loads(line) 
                except (UnicodeDecodeError, json.JSONDecodeError):
                    pass
            
            # 5. Read Arm (Placeholder - insert your arm's read function here)
            arm_state = {"joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

            # 6. Save Data synchronously
            img_name = f"frame_{timestamp:.3f}.jpg"
            cv2.imwrite(f"{OUTPUT_DIR}/images/{img_name}", frame)
            
            log_entry = {
                "timestamp": timestamp,
                "image_file": img_name,
                "chassis": chassis_state,
                "arm": arm_state
            }
            
            log_file.write(json.dumps(log_entry) + "\n")
            
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Logged {frame_count} synced frames...")
                
            # Cap framerate to ~30 Hz to prevent flooding the CPU/Disk
            time.sleep(max(0, 0.033 - (time.time() - timestamp)))

    except KeyboardInterrupt:
        print("\nStopping data collection...")
    finally:
        cap.release()
        if esp32:
            esp32.close()
        log_file.close()
        print("Files saved and hardware safely released.")

if __name__ == "__main__":
    main()