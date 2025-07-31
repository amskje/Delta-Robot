import cv2
from ultralytics import YOLO
from dataclasses import dataclass
import numpy as np
import threading

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
    CONF_THRESHOLD: float = 0.8

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
        self.camera_cal = np.load("camera_calibration.npz")
        self.mtx = self.camera_cal['camera_matrix']
        self.dist = self.camera_cal['dist_coeffs']
        self.H = np.load("homography_camera.npy")

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

#cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
latest_frame = None
frame_lock = threading.Lock()

# === FUNCTIONS ===

def start_camera_thread():
    global latest_frame
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Faailed to open camera")
        return
    def capture_loop():
        global latest_frame
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Faield to grab frame")
                continue
            with frame_lock:
                latest_frame = frame.copy()
    thread = threading.Thread(target=capture_loop, daemon=True)
    thread.start()

def get_camera():
    global cap
    if not cap.isOpened():
        cap.open(gst_pipeline)
    return cap 

def init_yolo(model_path="modules/best.pt"):
    """Initialize and return the YOLO model."""
    return YOLO(model_path)

def pixel_to_world(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pt, H)
    x_mm, y_mm = world_pt[0][0]
    dx, dy = 5.975, -5.975
    return x_mm - dx, y_mm - dy

def detect_target(model, target_class, mtx, dist, H):
    """
    Capture one frame and return all matching objects of the given class
    as (class_name, x_cm, y_cm, confidence) tuples.
    """
    start = time.perf_counter()

    #cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    #if not cap.isOpened():
     #   print("Failed to open camera")
      #  return []

    global latest_frame 
    with frame_lock:
        if latest_frame is None:
            print("No frame avalible yet")
            return[]
        frame = latest_frame.copy()

    #cap = get_camera()
    #ret, frame = cap.read()
    #cap.release()
    print("frame capture time:", time.perf_counter()-start)

    #if not ret:
     #   print("Failed to capture frame.")
      #  return []

    # Undistort
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    start = time.perf_counter()
    results = model(undistorted, conf=config().CONF_THRESHOLD)[0]
    print("YOLO inference time:", time.perf_counter()-start)

 
    annotated_frame = results.plot()

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

        # Draw red center dot
        cv2.circle(annotated_frame, (x_pixel, y_pixel), radius=5, color=(0, 0, 255), thickness=-1)

        x_mm_old = (x_pixel - config().IMG_CENTER_X) * config().PIXEL_TO_CM_X * 10  # Convert to mm
        y_mm_old = (y_pixel - config().IMG_CENTER_Y) * config().PIXEL_TO_CM_Y * 10  # Convert to mm

        x_mm, y_mm = pixel_to_world(x_pixel, y_pixel, H)

        # Overlay coordinate text
        coord_label = f"({x_mm:.1f}, {y_mm:.1f}) mm"
        cv2.putText(annotated_frame, coord_label, (x_pixel + 5, y_pixel - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        print(f"Undistorted Real-world coords: X = {x_mm_old:.2f} mm, Y = {y_mm_old:.2f} mm")
        print(f"H and undistorted Real-world coords: X = {x_mm:.2f} mm, Y = {y_mm:.2f} mm")

        matches.append((class_name, x_mm, y_mm, conf))

    # TESTING FOR Å SE FORSKJELL PÅ GAMMLE OG NYE, CALIBRERTE VERDIER
    results_old = model(frame, conf=config().CONF_THRESHOLD)[0]

    for box in results_old.boxes:
        class_id = int(box.cls[0].item())
        class_name = class_names[class_id]
        conf = float(box.conf[0].item())

        if class_name != target_class:
            print("Mismatch class name")
            continue

        x_pixel = int(box.xywh[0][0].item())
        y_pixel = int(box.xywh[0][1].item())

        x_mm = (x_pixel - config().IMG_CENTER_X) * config().PIXEL_TO_CM_X * 10  # Convert to mm
        y_mm = (y_pixel - config().IMG_CENTER_Y) * config().PIXEL_TO_CM_Y * 10  # Convert to mm

        print(f"Old Real-world coords: X = {x_mm:.2f} mm, Y = {y_mm:.2f} mm")

    if matches:
        cv2.imshow("Picture", annotated_frame)
        cv2.waitKey(1000000)
        cv2.destroyWindow("Picture")

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

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to read frame.")
            continue

        draw_overlay(frame)
        cv2.imshow("🍬 YOLOv8 Live Detection", frame)

        # Handle 'q' key press in video window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cap.release()
    cv2.destroyAllWindows()