import os
import sys
import serial
import time

# Importing custom modules
import modules.comms as comms
import modules.control as control
import modules.kinematics as kinematics
import modules.vision as vision

# Configurations
comCFG = comms.config()  # Configs from comms module
conCFG = control.config() # Configs from control module
kinCFG = kinematics.config() # Configs from kinematics module
visCFG = vision.config() # Configs from vision module

current_position = conCFG.INITIAL_POSITION  # Initial position after goHome()

def main():
    global current_position

    try:
        ser = serial.Serial(comCFG.SERIAL_PORT, comCFG.BAUD_RATE, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    if not comms.wait_for_arduino_ready(ser):
        ser.close()
        return

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

            print(f"Planning move to ({x}, {y}, {z}) cm...")
            angles = []
            kinematics.plan_linear_move(
                current_position.x, current_position.y, current_position.z,
                x, y, z,
                angles,
                conCFG.WAYPOINTS
            )

            theta1, theta2, theta3 = kinematics.inverse_kinematics(x*10, y*10, z*10)

            if angles:
                comms.send_to_arduino(ser, angles)
                current_position = kinematics.Position(x, y, z)
            else:
                print("No valid angles generated. Target might be unreachable.")

        except ValueError:
            print("Invalid input. Please enter 3 numbers separated by spaces.")

    ser.close()

if __name__ == '__main__':
    main()
