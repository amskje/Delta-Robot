import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ==== CONFIGURATION ====
FRAME_WIDTH = 640
FRAME_HEIGHT = 640
SURFACE_WIDTH_CM = 29.6
SURFACE_HEIGHT_CM = 29.6
CONF_THRESHOLD = 0.5
MODEL_PATH = "best.pt"
SEND_FIRST_ONLY = True  # Only send one candy per frame
SERIAL_TOPIC = 'arduino_command'
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

def main():
    # Start ROS
    rclpy.init()
    node = rclpy.create_node('yolo_coordinate_publisher')
    publisher = node.create_publisher(String, SERIAL_TOPIC, 10)

    # Open camera
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("🚫 Failed to open camera.")
        rclpy.shutdown()
        return

    print("🎯 YOLO detection running and publishing to Arduino...")

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Failed to read frame.")
                break

            # YOLO inference
            results = model(frame, conf=CONF_THRESHOLD)[0]

            for i, box in enumerate(results.boxes):
                x_pixel = int(box.xywh[0][0].item())
                y_pixel = int(box.xywh[0][1].item())

                x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
                y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

                # Send formatted string "x_cm,y_cm" to Arduino
                msg = String()
                msg.data = f"{x_cm:.1f},{y_cm:.1f}"
                publisher.publish(msg)
                node.get_logger().info(f"Sent: {msg.data}")

                if SEND_FIRST_ONLY:
                    break  # Only send one candy per frame

    except KeyboardInterrupt:
        print("🛑 Detection stopped by user.")

    cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
