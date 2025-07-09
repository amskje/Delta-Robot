import sys
import os
import serial
import time

# Add the 'modules' folder to Python's import path
sys.path.append(os.path.join(os.path.dirname(__file__), "modules"))

import cv2
from ultralytics import YOLO
import kinematics

# ==== CONFIGURATION ====
FRAME_WIDTH = 640
FRAME_HEIGHT = 640
SURFACE_WIDTH_CM = 29.6
SURFACE_HEIGHT_CM = 29.6
CONF_THRESHOLD = 0.9
MODEL_PATH = "best.pt"
SEND_FIRST_ONLY = True
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 57600
# ========================

PIXEL_TO_CM_X = SURFACE_WIDTH_CM / FRAME_WIDTH
PIXEL_TO_CM_Y = SURFACE_HEIGHT_CM / FRAME_HEIGHT
IMG_CENTER_X = FRAME_WIDTH // 2
IMG_CENTER_Y = FRAME_HEIGHT // 2

class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "karamell", "Lakris", "Notti", "Toffee", "Eclairs"
]

# Load YOLO model
model = YOLO(MODEL_PATH)

# GStreamer pipeline
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

current_position = Position(3.373, 0.184, 257.886)  # Initial calibrated position
angles = []

def read_serial_responses(ser):
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino]: {line}")
        except UnicodeDecodeError:
            pass  # Skip malformed lines


def main():
    # Open serial connection to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)  # wait for Arduino reset
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
            read_serial_responses(ser)  # ← Add this to check serial responses

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
                results = model(latest_frame, conf=CONF_THRESHOLD)[0]

                if not results.boxes:
                    print("No object detected.")
                    continue

                box = results.boxes[0]
                x_pixel = int(box.xywh[0][0].item())
                y_pixel = int(box.xywh[0][1].item())
                x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
                y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

                angles.clear()
                kinematics.plan_linear_move(current_position.x, current_position.y, current_position.z, x_cm, y_cm, 200, angles)

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
