
import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

# Values: 128 = Neutral, 0 = Full Left / Full Reverse, 255 = Full Right / Full Forward
NEUTRAL = 128

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Wait for ESP32 reset after serial connection
    print(f"Connected to {PORT}.")
    print("\nControls:")
    print("  a : Steer Left   (80)")
    print("  d : Steer Right  (175)")
    print("  c : Center Steer (128)")
    print("  w : Pulse Forward (160 for 0.4s)")
    print("  s : Emergency Stop / All Neutral")
    print("  m : Manual input (custom steer,throttle)")
    print("  q : Quit\n")

    current_steer = NEUTRAL
    current_throt = NEUTRAL

    def send(s, t):
        ser.write(f"{s},{t}\n".encode('utf-8'))
        time.sleep(0.05)
        if ser.in_waiting:
            print("ESP32:", ser.readline().decode('utf-8').strip())

    # Initialize to neutral
    send(current_steer, current_throt)

    while True:
        cmd = input("Command > ").strip().lower()

        if cmd == 'a':
            current_steer = 80
            print("Steering: Left")
            send(current_steer, current_throt)
        elif cmd == 'd':
            current_steer = 175
            print("Steering: Right")
            send(current_steer, current_throt)
        elif cmd == 'c':
            current_steer = NEUTRAL
            print("Steering: Centered")
            send(current_steer, current_throt)
        elif cmd == 'w':
            print("Throttling forward...")
            send(current_steer, 160)
            time.sleep(0.4)
            send(current_steer, NEUTRAL)
            print("Throttle back to neutral")
        elif cmd == 's':
            current_steer = NEUTRAL
            current_throt = NEUTRAL
            send(NEUTRAL, NEUTRAL)
            print("Stopped.")
        elif cmd == 'm':
            val = input("Enter 'steer,throttle' (e.g. 100,128): ").strip()
            if ',' in val:
                s, t = val.split(',')
                current_steer = int(s)
                current_throt = int(t)
                send(current_steer, current_throt)
        elif cmd == 'q':
            send(NEUTRAL, NEUTRAL)
            break

except serial.SerialException as e:
    print(f"Port error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.write(b"128,128\n")
        ser.close()
    print("Test session closed.")