import serial

try:
    esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Listening to ESP32... (Press Ctrl+C to quit)")
    while True:
        if esp32.in_waiting > 0:
            raw_bytes = esp32.readline()
            print(f"RAW: {raw_bytes}")
            try:
                text = raw_bytes.decode('utf-8').strip()
                print(f"TEXT: {text}")
            except Exception as e:
                print(f"Decode error: {e}")
except Exception as e:
    print(f"Could not open port: {e}")