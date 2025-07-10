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
WAYPOINTS = 5  # Remember that the kinematics module has a MAX_waypoints constant
# ========================

# Insert target XYZ coordinates here (in mm)
target_x = 0.0
target_y = 0.0
target_z = 25.0

# Calibrated current position
current_position = kinematics.Position(3.373, 0.184, 257.886)

def send_to_arduino(angles_list):
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)
        print("Serial connected to Arduino.")
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    # Clear initial serial messages
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino]: {line}")
        except:
            pass

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

    ser.close()

def main():
    global current_position
    print("Delta Robot Control Interface")
    print("Enter coordinates as: X Y Z  (in mm)")
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

            print(f"Planning move to ({x}, {y}, {z}) mm...")
            angles = []
            kinematics.plan_linear_move(
                current_position.x, current_position.y, current_position.z,
                x, y, z,
                angles,
                waypoints=WAYPOINTS
            )

            if angles:
                send_to_arduino(angles)
                # Update current position
                current_position = kinematics.Position(x, y, z)
            else:
                print("No valid angles generated. Target might be unreachable.")

        except ValueError:
            print("Invalid input. Please enter 3 numbers separated by spaces.")

if __name__ == '__main__':
    main()