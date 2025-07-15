import cv2
from ultralytics import YOLO
from dataclasses import dataclass
import os
import time

@dataclass
class VisionConfig:
    # Base parameters
    FRAME_WIDTH: int = 640
    FRAME_HEIGHT: int = 640
    SURFACE_WIDTH_CM: float = 29.6
    SURFACE_HEIGHT_CM: float = 29.6
    CAM_TO_ROBOT_Y_OFFSET_CM: float = 1.6
    MODEL_PATH: str = "modules/best.pt"
    CONF_THRESHOLD: float = 0.9

    # Derived parameters will be computed
    PIXEL_TO_CM_X: float = None
    PIXEL_TO_CM_Y: float = None
    CM_TO_PIXEL_X: float = None
    CM_TO_PIXEL_Y: float = None
    IMG_CENTER_X: int = None
    IMG_CENTER_Y: int = None

    def __post_init__(self):
        # Calculate derived values
        self.PIXEL_TO_CM_X = self.SURFACE_WIDTH_CM / self.FRAME_WIDTH
        self.PIXEL_TO_CM_Y = self.SURFACE_HEIGHT_CM / self.FRAME_HEIGHT
        self.CM_TO_PIXEL_X = self.FRAME_WIDTH / self.SURFACE_WIDTH_CM
        self.CM_TO_PIXEL_Y = self.FRAME_HEIGHT / self.SURFACE_HEIGHT_CM
        self.IMG_CENTER_X = self.FRAME_WIDTH // 2
        self.IMG_CENTER_Y = int((self.FRAME_HEIGHT // 2) - (self.CAM_TO_ROBOT_Y_OFFSET_CM / self.PIXEL_TO_CM_Y))

def config() -> VisionConfig:
    return VisionConfig()


# GStreamer pipeline
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={config().FRAME_WIDTH}, height={config().FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Class names
class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "Karamell", "Lakris", "Notti", "Toffee", "Eclairs"
]

# === FUNCTIONS ===

def init_yolo(model_path="modules/best.pt"):
    """Initialize and return the YOLO model."""
    return YOLO(model_path)

def detect_target(model, target_class):
    """
    Capture one frame and return all matching objects of the given class
    as (class_name, x_cm, y_cm, confidence) tuples.
    """

    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera.")
        return []


    #Test
    for _ in range(40):
        ret, _ = cap.read()
    #Test

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame.")
        return []

    results = model(frame, conf=config().CONF_THRESHOLD)[0]
    matches = []

    for box in results.boxes:
        class_id = int(box.cls[0].item())
        class_name = class_names[class_id]
        conf = float(box.conf[0].item())

        if class_name != target_class:
            print("Mismatch class name")
            continue

        x_pixel = int(box.xywh[0][0].item())
        y_pixel = int(box.xywh[0][1].item())

        x_cm = (x_pixel - config().IMG_CENTER_X) * config().PIXEL_TO_CM_X
        y_cm = (y_pixel - config().IMG_CENTER_Y) * config().PIXEL_TO_CM_Y

        matches.append((class_name, x_cm, y_cm, conf))

    return matches

def video_without_inference():
    """Display the live video feed without running inference."""
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    cv2.namedWindow("Live Feed", cv2.WINDOW_AUTOSIZE)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow("Live Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Draw overlay on video frame
def draw_overlay(frame):
    # Draw center dot (robot origin)
    cv2.circle(frame, (config().IMG_CENTER_X, config().IMG_CENTER_Y), radius=2, color=(255, 0, 0), thickness=-1)

    # Grid overlay for testing
    for i in range(1, 13):
        cv2.circle(frame, (config().IMG_CENTER_X + int(i * 21.62), config().IMG_CENTER_Y), radius=2, color=(0, 255, 0), thickness=-1)
        cv2.circle(frame, (config().IMG_CENTER_X - int(i * 21.62), config().IMG_CENTER_Y), radius=2, color=(0, 255, 0), thickness=-1)
    for j in range(1, 13):
        cv2.circle(frame, (config().IMG_CENTER_X, config().IMG_CENTER_Y + int(j * 21.62)), radius=2, color=(0, 255, 0), thickness=-1)
        cv2.circle(frame, (config().IMG_CENTER_X, config().IMG_CENTER_Y - int(j * 21.62)), radius=2, color=(0, 255, 0), thickness=-1)

# Video thread
def video_loop(cap, stop_event):
    global running
    cv2.namedWindow("🍬 YOLOv8 Live Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("🍬 YOLOv8 Live Detection", config().FRAME_WIDTH, config().FRAME_HEIGHT)

    # Setup for frame saving
    save_dir = "/home/jetson/yolo_frames"  # You can change this path
    os.makedirs(save_dir, exist_ok=True)
    last_saved_time = 0

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to read frame.")
            continue

        cv2.imshow("🍬 YOLOv8 Live Detection", frame)

        # Save one frame every two seconds
        current_time = time.time()
        if current_time - last_saved_time >= 2.0:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            frame_path = os.path.join(save_dir, f"frame_{timestamp}.jpg")
            cv2.imwrite(frame_path, frame)
            last_saved_time = current_time

        # Handle 'q' key press in video window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cap.release()
    cv2.destroyAllWindows()