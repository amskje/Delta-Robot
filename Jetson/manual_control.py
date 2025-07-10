import os
import sys

# Add the 'modules' folder to Python's import path
sys.path.append(os.path.join(os.path.dirname(__file__), "modules"))

import serial
import time
import kinematics
from kinematics import Position

# ==== CONFIGURATION ====
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 57600
WAYPOINTS = 8
# ========================

current_position = kinematics.Position(3.373, 0.184, 257.886)

def wait_for_arduino_ready(ser):
    print("Waiting for Arduino to finish setup...")
    deadline = time.time() + 10  # timeout after 10 seconds
    while time.time() < deadline:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino]: {line}")
                if "Finished setup" in line:
                    print("Arduino is ready.\n")
                    return True
        except:
            pass
    print("Timed out waiting for Arduino.")
    return False

def send_to_arduino(ser, angles_list):
    # Flush any remaining input
    ser.reset_input_buffer()

    print("Sending angles to Arduino...")
    ser.write(b"POSITION\n")
    ser.flush()
    time.sleep(0.1)

    for idx, angle_set in enumerate(angles_list):
        t1, t2, t3 = angle_set
        cmd = f"ANGLES {int(t1)},{int(t2)},{int(t3)}\n"
        print(f"Waypoint {idx + 1}: {cmd.strip()}")
        ser.write(cmd.encode())
        ser.flush()
        time.sleep(0.05)

    ser.write(b"GO\n")
    ser.flush()
    print("Movement command sent.\n")

def main():
    global current_position

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    if not wait_for_arduino_ready(ser):
        ser.close()
        return

    print("Delta Robot Control Interface")
    print("Enter coordinates as: X Y Z  (in cm)")
    print("Type 'q' to quit.\n")

    while True:
        user_input = input("Enter target X Y Z: ").strip()

        if user_input.lower() == 'q':
            print("Exiting.")
            break

        try:
            parts = user_input.split()
            if len(parts) != 3:
                raise ValueError("Expected three values")

            x, y, z = map(float, parts)
            print(f"Planning move to ({x}, {y}, {z}) cm...")

            # Convert to mm for inverse kinematics
            angles = []
            kinematics.plan_linear_move(
                current_position.x * 10, current_position.y * 10, current_position.z * 10,
                x * 10, y * 10, z * 10,
                angles,
                waypoints=WAYPOINTS
            )

            if angles:
                send_to_arduino(ser, angles)
                current_position = Position(x, y, z)
            else:
                print("No valid angles generated. Target might be unreachable.")

        except ValueError:
            print("Invalid input. Please enter 3 numbers separated by spaces.")

    ser.close()

if __name__ == '__main__':
    main()
