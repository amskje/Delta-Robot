import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Delta_pathplanning as dp

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

class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

current_position = Position()
# Initialize current position (x, y, z) as the one after homing is called on arduino. Limitswitches -100 steps. Forward kinematics from https://www.marginallyclever.com/other/samples/fk-ik-test.html
current_position.x = 3.373
current_position.y = 0.184
current_position.z = 257.886

angles = []


def main():
    # Start ROS
    rclpy.init()
    node = rclpy.create_node('yolo_coordinate_publisher')
    publisher = node.create_publisher(String, SERIAL_TOPIC, 10)

    # Open camera
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera.")
        rclpy.shutdown()
        return

    print("Press 'b' to capture and send one detection. Press 'q' to quit.")

    try:
        while rclpy.ok():
            key = cv2.waitKey(1) & 0xFF
            if key == ord('b'):
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read frame.")
                    continue

                results = model(frame, conf=CONF_THRESHOLD)[0]

                if not results.boxes:
                    print("No object detected.")
                    continue

                # Use only the first detected object
                box = results.boxes[0]
                x_pixel = int(box.xywh[0][0].item())
                y_pixel = int(box.xywh[0][1].item())

                x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
                y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

                dp.plan_linear_move(current_position.x, current_position.y, current_position.z, x_cm, y_cm, 250, angles)

                msg = String()
                msg.data = f"\nPOSITION"
                publisher.publish(msg)
                node.get_logger().info(f"Sent: {msg.data}")

                for angle_set in angles:
                    t1, t2, t3 = angle_set
                    msg = String()
                    msg.data = f"\nANGLES {t1:.2f},{t2:.2f},{t3:.2f}"
                    publisher.publish(msg)
                    node.get_logger().info(f"Sent: {msg.data}")

                msg = String()
                msg.data = f"\nGO"
                publisher.publish(msg)
                node.get_logger().info(f"Sent: {msg.data}")
                


            elif key == ord('q'):
                print("Quitting.")
                break

    except KeyboardInterrupt:
        print("Detection stopped by user.")

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()