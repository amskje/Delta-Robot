import os
import sys
import serial
import time
import threading
import cv2

# Add 'modules' folder to path
sys.path.append(os.path.join(os.path.dirname(__file__), "modules"))
import kinematics
from kinematics import Position

# ==== CONFIGURATION ====
FRAME_WIDTH = 640
FRAME_HEIGHT = 640
SURFACE_WIDTH_CM = 29.6
SURFACE_HEIGHT_CM = 29.6
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 57600
WAYPOINTS = 8
CAM_TO_ROBOT_Y_OFFSET_CM = 1.6
# ========================

# Derived conversions
PIXEL_TO_CM_X = SURFACE_WIDTH_CM / FRAME_WIDTH
PIXEL_TO_CM_Y = SURFACE_HEIGHT_CM / FRAME_HEIGHT
CM_TO_PIXEL_X = FRAME_WIDTH / SURFACE_WIDTH_CM
CM_TO_PIXEL_Y = FRAME_HEIGHT / SURFACE_HEIGHT_CM
IMG_CENTER_X = FRAME_WIDTH // 2
IMG_CENTER_Y = int((FRAME_HEIGHT // 2) - (CAM_TO_ROBOT_Y_OFFSET_CM / PIXEL_TO_CM_Y))

# Global control flag
running = True

# GStreamer pipeline
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Draw overlay on video frame
def draw_overlay(frame):
    # Draw center dot (robot origin)
    cv2.circle(frame, (IMG_CENTER_X, IMG_CENTER_Y), radius=2, color=(255, 0, 0), thickness=-1)

    # Grid overlay for testing
    for i in range(1, 13):
        cv2.circle(frame, (IMG_CENTER_X + int(i * 21.62), IMG_CENTER_Y), radius=2, color=(0, 255, 0), thickness=-1)
        cv2.circle(frame, (IMG_CENTER_X - int(i * 21.62), IMG_CENTER_Y), radius=2, color=(0, 255, 0), thickness=-1)
    for j in range(1, 13):
        cv2.circle(frame, (IMG_CENTER_X, IMG_CENTER_Y + int(j * 21.62)), radius=2, color=(0, 255, 0), thickness=-1)
        cv2.circle(frame, (IMG_CENTER_X, IMG_CENTER_Y - int(j * 21.62)), radius=2, color=(0, 255, 0), thickness=-1)

# Video thread
def video_loop(cap):
    global running
    cv2.namedWindow("🍬 YOLOv8 Live Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("🍬 YOLOv8 Live Detection", FRAME_WIDTH, FRAME_HEIGHT)

    while running:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to read frame.")
            continue

        draw_overlay(frame)
        cv2.imshow("🍬 YOLOv8 Live Detection", frame)

        # Handle 'q' key press in video window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
            break

    cap.release()
    cv2.destroyAllWindows()

# Arduino sync
def wait_for_arduino_ready(ser):
    print("Waiting for Arduino to finish setup...")
    deadline = time.time() + 12
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

# Send angles to Arduino
def send_to_arduino(ser, angles_list):
    ser.reset_input_buffer()
    print("Sending angles to Arduino...")
    ser.write(b"POSITION\n")
    ser.flush()
    time.sleep(0.1)

    for idx, angle_set in enumerate(angles_list):
        t1, t2, t3 = angle_set
        cmd = f"ANGLES {int(t1)},{int(t2)},{int(t3)}\n"
        ser.write(cmd.encode())
        ser.flush()
        time.sleep(0.05)

    ser.write(b"GO\n")
    ser.flush()
    print("Movement command sent.\n")

# Main control loop
def main():
    global running

    # Initialize camera
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("🚫 Failed to open camera.")
        return

    # Start video feed thread
    video_thread = threading.Thread(target=video_loop, args=(cap,))
    video_thread.start()

    # Connect to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)
    except Exception as e:
        print("Failed to open serial port:", e)
        running = False
        video_thread.join()
        return

    if not wait_for_arduino_ready(ser):
        ser.close()
        running = False
        video_thread.join()
        return

    current_position = Position(3.373, 0.184, 257.886)

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
                waypoints=WAYPOINTS
            )

            theta1, theta2, theta3 = kinematics.inverse_kinematics(x * 10, y * 10, z * 10)

            if angles:
                send_to_arduino(ser, angles)
                current_position = Position(x, y, z)
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
