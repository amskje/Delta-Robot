import cv2
import numpy as np
import json
import time
import threading
import serial
from threading import Event

# Custom modules
import modules.comms as comms
import modules.control as control
import modules.kinematics as kinematics
import modules.vision as vision

# Configurations
comCFG = comms.config()
conCFG = control.config()
kinCFG = kinematics.config()
visCFG = vision.config()

def correct_target(x_desired, y_desired, H_inv):
    pt = np.array([[[x_desired, y_desired]]], dtype=np.float32)
    corrected = cv2.perspectiveTransform(pt, H_inv)
    return corrected[0][0]

def main():
    running = True

    # === Start Vision Inference ===
    vision_state = vision.VisionState()
    model = vision.init_yolo(visCFG.MODEL_PATH)

    vision.start_inference_thread(model, visCFG, vision_state)
    vision.wait_for_inference_ready(vision_state)
    
    # Start detection display window (optional but useful)
    display_thread = threading.Thread(target=vision.show_live_detections, args=(vision_state,), daemon=True)
    display_thread.start()

    # === Start Serial Connection to Arduino ===
    try:
        ser = serial.Serial(comCFG.SERIAL_PORT, comCFG.BAUD_RATE, timeout=1)
        time.sleep(2)
    except Exception as e:
        print("❌ Failed to open serial port:", e)
        return

    # Restart Arduino
    ser.dtr = False
    time.sleep(1)
    ser.dtr = True
    time.sleep(1)

    if not comms.wait_for_arduino_ready(ser):
        ser.close()
        return

    current_position = conCFG.INITIAL_POSITION

    print("\n🎮 Delta Robot Control Interface")
    print("Type X Y Z coordinates in mm (or 'q' to quit)")

    with open('homography_ROBOT_WORLD.json', 'r') as file:
        H_robot = json.load(file)
    H_robot_inv = np.linalg.inv(H_robot)

    while running:
        try:
            user_input = input("📍 Target X Y Z > ").strip()
            if user_input.lower() == 'q':
                break

            parts = user_input.split()
            if len(parts) != 3:
                print("❌ Invalid input. Format: X Y Z")
                continue

            x, y, z = map(float, parts)
            print(f"[Control] Planning move to ({x}, {y}, {z}) mm...")

            angles = []
            x_corr, y_corr = correct_target(x, y, H_robot_inv)

            kinematics.plan_linear_move(
                current_position.x, current_position.y, current_position.z,
                x_corr, y_corr, z,
                angles,
                conCFG.WAYPOINTS
            )

            if angles:
                comms.send_to_arduino(ser, angles)
                current_position = kinematics.Position(x_corr, y_corr, z)
            else:
                print("⚠️ No valid angles generated — point may be unreachable.")

        except ValueError:
            print("❌ Please enter 3 valid numbers (X Y Z)")

    print("🛑 Shutting down...")
    ser.close()
    print("👋 Goodbye!")

if __name__ == '__main__':
    main()
