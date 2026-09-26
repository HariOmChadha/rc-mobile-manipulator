import serial
import time
import csv

PORT = '/dev/ttyUSB0' # Ensure this matches your ESP32 port
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}. Logging data... Press Ctrl+C to stop.")
    
    with open('rc_log.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'Steering_DAC', 'Throttle_DAC'])
        
        while True:
            if ser.in_waiting > 0:
                # Read the line from ESP32 and decode it
                raw_data = ser.readline().decode('utf-8').strip()
                
                # Split the "128,128" string into two variables
                if "," in raw_data:
                    steer, throttle = raw_data.split(',')
                    current_time = time.time()
                    
                    # Print to terminal
                    print(f"Steer: {steer} | Throttle: {throttle}")
                    
                    # Save to CSV
                    writer.writerow([current_time, steer, throttle])
                    f.flush() # Force write to disk immediately

except serial.SerialException as e:
    print(f"Error opening port: {e}")
except KeyboardInterrupt:
    print("\nLogging stopped. Data saved to rc_log.csv.")
    ser.close()