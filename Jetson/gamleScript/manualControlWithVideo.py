# Importing packages
import os
import sys
import serial
import time
import threading
import cv2
from threading import Event
import json
import numpy as np

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


def correct_target(x_desired, y_desired, H_inv):
    """Transform a target point from real-world into robot coordinates"""
    pt = np.array([[[x_desired, y_desired]]], dtype=np.float32)
    corrected = cv2.perspectiveTransform(pt, H_inv)
    return corrected[0][0]

# Main loop with functions from here and there
def main():
    global running

    # Initialize camera
    cap = cv2.VideoCapture(vision.gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("🚫 Failed to open camera.")
        return

    # Start video feed thread
    stop_event = Event()
    video_thread = threading.Thread(target=vision.video_loop, args=(cap, stop_event))
    video_thread.start()

    # Connect to Arduino
    try:
        ser = serial.Serial(comCFG.SERIAL_PORT, comCFG.BAUD_RATE, timeout=1)
        time.sleep(2)
    except Exception as e:
        print("Failed to open serial port:", e)
        stop_event.set()
        video_thread.join()
        return

    # Restart Arduino
    ser.dtr = False
    time.sleep(1)
    ser.dtr = True
    time.sleep(1)

    if not comms.wait_for_arduino_ready(ser):
        ser.close()
        stop_event.set()
        video_thread.join()
        return

    current_position = conCFG.INITIAL_POSITION  # Initial position after goHome()

    print("Delta Robot Control Interface")
    print("Enter coordinates as: X Y Z  (in mm)")
    print("Type 'q' to quit.\n")

    with open('homography_ROBOT_WORLD.json', 'r') as file:
            H_robot = json.load(file)

    H_robot_inv = np.linalg.inv(H_robot)
    


    while running:
        try:
            user_input = input("Enter target X Y Z: ").strip()
            if user_input.lower() == 'q':
                break

            parts = user_input.split()
            if len(parts) != 3:
                raise ValueError("Expected three values")

            x, y, z = map(float, parts)
            print(f"Planning move to ({x}, {y}, {z}) mm...")

            angles = []

            x_corrected, y_corrected = correct_target(x, y, H_robot_inv)

            kinematics.plan_linear_move(
                current_position.x, current_position.y, current_position.z,
                x_corrected, y_corrected, z,
                angles,
                conCFG.WAYPOINTS
            )

            if angles:
                comms.send_to_arduino(ser, angles)
                current_position = kinematics.Position(x_corrected, y_corrected, z)
            else:
                print("No valid angles generated. Target might be unreachable.")

        except ValueError:
            print("Invalid input. Please enter 3 numbers separated by spaces.")

    print("Shutting down...")
    ser.close()
    stop_event.set()
    running = False
    video_thread.join()
    print("Goodbye!")

if __name__ == '__main__':
    main()
