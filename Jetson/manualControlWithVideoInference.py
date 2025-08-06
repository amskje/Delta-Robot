import cv2
import numpy as np
import json
import time
import threading
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

    # === Start display window for detections
    display_thread = threading.Thread(
        target=vision.show_live_detections,
        args=(vision_state,),
        daemon=True
    )
    display_thread.start()

    # === Start Serial Connection to Arduino using SerialComm
    serial_comm = comms.SerialComm()

    # === Wait for Arduino to confirm setup
    if not comms.wait_for_arduino_ready(serial_comm.conn):
        print("❌ Arduino did not respond. Aborting.")
        serial_comm.close()
        return

    # === Load homography
    with open('homography_ROBOT_WORLD.json', 'r') as file:
        H_robot = json.load(file)
    H_robot_inv = np.linalg.inv(H_robot)

    # === Create controller with serial_comm
    controller = control.DeltaRobotController(serial_comm)

    # === Move robot to photo position
    print("[Control] Moving robot to camera view position...")
    controller.go_to_pos(move_pos=(0, 0, -305))
    controller.go_to_pos(move_pos=(-120, 80, -305))

    # === Robot control loop
    current_position = conCFG.INITIAL_POSITION

    print("\n🎮 Delta Robot Control Interface")
    print("Type X Y Z coordinates in mm (or 'q' to quit)")

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

            # Apply homography correction
            x_corr, y_corr = correct_target(x, y, H_robot_inv)

            angles = []
            kinematics.plan_linear_move(
                current_position.x, current_position.y, current_position.z,
                x_corr, y_corr, z,
                angles,
                conCFG.WAYPOINTS
            )

            if angles:
                comms.send_to_arduino(serial_comm.conn, angles)
                current_position = kinematics.Position(x_corr, y_corr, z)
            else:
                print("⚠️ No valid angles generated — point may be unreachable.")

        except ValueError:
            print("❌ Please enter 3 valid numbers (X Y Z)")

    print("🛑 Shutting down...")
    serial_comm.close()
    print("👋 Goodbye!")

if __name__ == '__main__':
    main()
