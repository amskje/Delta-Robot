import sys
import os
import serial
import time
import cv2
from ultralytics import YOLO

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

class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "karamell", "Lakris", "Notti", "Toffee", "Eclairs"
]

# Load YOLO model
model = YOLO(visCFG.MODEL_PATH)

# GStreamer pipeline
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={visCFG.FRAME_WIDTH}, height={visCFG.FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)


current_position = conCFG.INITIAL_POSITION # Initial calibrated position
angles = []




def main():
    # Open serial connection to Arduino
    try:
        ser = comms.SerialComm(comCFG.SERIAL_PORT, comCFG.BAUD_RATE, timeout=1).conn
        print("Serial connected to Arduino.")
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    # Open camera
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    cv2.namedWindow("YOLO Trigger Window", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("YOLO Trigger Window", 200, 100)

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    print("Press 'b' to capture and send one detection. Press 'q' to quit.")

    try:
        while True:
            comms.SerialComm.read_serial_responses(ser)  # ← Add this to check serial responses

            key = cv2.waitKey(1) & 0xFF
            if key == ord('b'):
                # Capture multiple frames and use the last one
                latest_frame = None
                for _ in range(10):
                    ret, frame = cap.read()
                    if not ret:
                        print("Failed to read frame.")
                        break
                    latest_frame = frame

                if latest_frame is None:
                    continue

                cv2.imshow("YOLO Trigger Window", latest_frame)
                results = model(latest_frame, conf=visCFG.CONF_THRESHOLD)[0]

                if not results.boxes:
                    print("No object detected.")
                    continue

                box = results.boxes[0]
                x_pixel = int(box.xywh[0][0].item())
                y_pixel = int(box.xywh[0][1].item())
                x_cm = (x_pixel - visCFG.IMG_CENTER_X) * visCFG.PIXEL_TO_CM_X
                y_cm = (y_pixel - visCFG.IMG_CENTER_Y) * visCFG.PIXEL_TO_CM_Y

                angles.clear()
                kinematics.plan_linear_move(current_position.x, current_position.y, current_position.z, x_cm, y_cm, visCFG.Z_HEIGHT_CM, angles)

                ser.write(b"POSITION\n")
                ser.flush()
                time.sleep(0.1)

                for idx, angle_set in enumerate(angles):
                    t1, t2, t3 = angle_set
                    cmd = f"ANGLES {int(t1)},{int(t2)},{int(t3)}\n"
                    print(f"Waypoint {idx + 1}: {cmd.strip()}")
                    ser.write(cmd.encode())
                    ser.flush()
                    time.sleep(0.05)

                ser.write(b"GO\n")
                ser.flush()
                print("Sent detection and movement command to Arduino.")

            elif key == ord('q'):
                print("Quitting.")
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == '__main__':
    main()
