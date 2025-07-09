import cv2
from ultralytics import YOLO

# ==== CONFIGURATION ====
FRAME_WIDTH = 640
FRAME_HEIGHT = 640
SURFACE_WIDTH_CM = 29.6
SURFACE_HEIGHT_CM = 29.6
CONF_THRESHOLD = 0.5

PIXEL_TO_CM_X = SURFACE_WIDTH_CM / FRAME_WIDTH
PIXEL_TO_CM_Y = SURFACE_HEIGHT_CM / FRAME_HEIGHT
IMG_CENTER_X = FRAME_WIDTH // 2
IMG_CENTER_Y = FRAME_HEIGHT // 2

# GStreamer pipeline
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Class names
class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "karamell", "Lakris", "Notti", "Toffee", "Eclairs"
]

# === FUNCTIONS ===

def init_yolo(model_path="best.pt"):
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

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame.")
        return []

    results = model(frame, conf=CONF_THRESHOLD)[0]
    matches = []

    for box in results.boxes:
        class_id = int(box.cls[0].item())
        class_name = class_names[class_id]
        conf = float(box.conf[0].item())

        if class_name != target_class:
            continue

        x_pixel = int(box.xywh[0][0].item())
        y_pixel = int(box.xywh[0][1].item())

        x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
        y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

        matches.append((class_name, x_cm, y_cm, conf))

    return matches
