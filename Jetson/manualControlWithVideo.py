# Importing packages
import os
import sys
import serial
import time
import threading
import cv2

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

# Global control flag
running = True

# Main loop with functions from here and there
def main():
    global running

    # Initialize camera
    cap = cv2.VideoCapture(vision.gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("🚫 Failed to open camera.")
        return

    # Start video feed thread
    video_thread = threading.Thread(target=vision.video_loop, args=(cap,))
    video_thread.start()

    # Connect to Arduino
    try:
        ser = serial.Serial(comCFG.SERIAL_PORT, comCFG.BAUD_RATE, timeout=1)
        time.sleep(2)
    except Exception as e:
        print("Failed to open serial port:", e)
        running = False
        video_thread.join()
        return

    if not comms.wait_for_arduino_ready(ser):
        ser.close()
        running = False
        video_thread.join()
        return

    current_position = kinematics.Position(3.373, 0.184, 257.886)

    print("Delta Robot Control Interface")
    print("Enter coordinates as: X Y Z  (in cm)")
    print("Type 'q' to quit.\n")

    while running:
        try:
            user_input = input("Enter target X Y Z: ").strip()
            if user_input.lower() == 'q':
                break

            parts = user_input.split()
            if len(parts) != 3:
                raise ValueError("Expected three values")

            x, y, z = map(float, parts)
            print(f"Planning move to ({x}, {y}, {z}) cm...")

            angles = []
            kinematics.plan_linear_move(
                current_position.x * 10, current_position.y * 10, current_position.z * 10,
                x * 10, y * 10, z * 10,
                angles,
                conCFG.WAYPOINTS
            )

            if angles:
                comms.send_to_arduino(ser, angles)
                current_position = kinematics.Position(x, y, z)
            else:
                print("No valid angles generated. Target might be unreachable.")

        except ValueError:
            print("Invalid input. Please enter 3 numbers separated by spaces.")

    print("Shutting down...")
    ser.close()
    running = False
    video_thread.join()
    print("Goodbye!")

if __name__ == '__main__':
    main()
