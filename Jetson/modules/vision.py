import cv2
import numpy as np
import threading
import time
import torch
from dataclasses import dataclass, field
from ultralytics import YOLO


@dataclass
class VisionConfig:
    # Base parameters
    FRAME_WIDTH: int = 640
    FRAME_HEIGHT: int = 640
    SURFACE_WIDTH_CM: float = 29.6
    SURFACE_HEIGHT_CM: float = 29.6
    CAM_TO_ROBOT_Y_OFFSET_CM: float = 1.6
    MODEL_PATH: str = "modules/weightsV3.pt"
    CONF_THRESHOLD: float = 0.7
    WORKING_RADIUS_CM: float = 14.8

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


@dataclass
class VisionState:
    latest_frame: np.ndarray = None
    latest_detections: list = field(default_factory=list)
    inference_ready: bool = False

    frame_lock: threading.Lock = field(default_factory=threading.Lock)
    detections_lock: threading.Lock = field(default_factory=threading.Lock)
    inference_ready_lock: threading.Lock = field(default_factory=threading.Lock)

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
    "Japp", "Karamell", "Lakris", "Notti", "Toffee", "Eclairs", "Marsipan"
]


# === FUNCTIONS ===

def start_inference_thread(model, config, state: VisionState):
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[Vision] ❌ GStreamer camera failed to open")
        return
    else:
        print("[Vision] ✅ GStreamer camera opened successfully")


    def inference_loop():
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Failed to read frame.")
                continue

            with state.frame_lock:
                state.latest_frame = frame.copy()

            # Undistort
            h, w = frame.shape[:2]
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(config.mtx, config.dist, (w, h), 1, (w, h))
            undistorted = cv2.undistort(frame, config.mtx, config.dist, None, newcameramtx)

            results = model(undistorted, device=0, verbose=False)[0]

            # Only set once
            with state.inference_ready_lock:
                if not state.inference_ready:
                    state.inference_ready = True
                    print("[Vision] ✅ Inference is now ready.")

            boxes_tensor = results.boxes.xyxy.cpu().numpy()
            confs_tensor = results.boxes.conf.cpu().numpy()
            class_ids_tensor = results.boxes.cls.cpu().numpy()

            detections = []
            for i in range(len(boxes_tensor)):
                x1, y1, x2, y2 = boxes_tensor[i].astype(int)
                conf = float(confs_tensor[i])
                class_id = int(class_ids_tensor[i])
                if conf > config.CONF_THRESHOLD:
                    detections.append((x1, y1, x2, y2, conf, class_id))
    
            with state.detections_lock:
                state.latest_detections.clear()
                state.latest_detections.extend(detections)

            for x1, y1, x2, y2, conf, class_id in detections:
                if class_id >= len(class_names):
                    print(f"[Vision] ⚠️ Skipping invalid class_id: {class_id}")
                    continue


    thread = threading.Thread(target=inference_loop, daemon=True)
    thread.start()
    print("[Vision] 🌀 Inference thread started")


def init_yolo(model_path="models/best.pt"):
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = YOLO(model_path)
    return model

def pixel_to_world(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pt, H)
    x_mm, y_mm = world_pt[0][0]
    dx, dy = 5.975, -5.975
    return x_mm - dx, y_mm - dy

def wait_for_inference_ready(state: VisionState, timeout=10.0):
    print("[Vision] ⏳ Waiting for inference to be ready...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        with state.inference_ready_lock:
            if state.inference_ready:
                print("[Vision] ✅ Inference is ready.")
                return True
        time.sleep(0.1)
    print("[Vision] ❌ Timeout: Inference was not ready in time.")
    return False


def detect_target(target_class, config: VisionConfig, state: VisionState):
    with state.detections_lock:
        detections = list(state.latest_detections)

    matches = []
    hit_shown = False

    for x1, y1, x2, y2, conf, class_id in detections:
        class_name = class_names[class_id]
        if class_name != target_class:
            continue
        
        bbox = (x1, y1, x2, y2, conf, class_id)
        refined_center = refine_center_by_ellipse(state.latest_frame, bbox, debug=False)


        # Compute center in pixels
        x_pixel = refined_center[0]
        y_pixel = refined_center[1]

        # Distance from image center
        dx = x_pixel - config.IMG_CENTER_X

        radius_px = int(config.WORKING_RADIUS_CM / config.PIXEL_TO_CM_X)

        dy = y_pixel - config.IMG_CENTER_Y
        dist_px = np.sqrt(dx**2 + dy**2)

        # Compare to radius (in px)
        if dist_px > radius_px:
            print(f"[Vision] Skipping {class_name} — outside working area.")
            continue  # outside of working area

        # Otherwise, compute world coords and add as match
        x_mm, y_mm = pixel_to_world(x_pixel, y_pixel, config.H)
        matches.append((class_name, x_mm, y_mm, conf))

        # Show current frame with hit visualization (use latest at hit moment)
        if not hit_shown:
            with state.frame_lock:
                frame = state.latest_frame.copy() if state.latest_frame is not None else None

            if frame is not None:
                
                label = f"{class_name} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(frame, (x_pixel, y_pixel), 4, (0, 0, 255), -1)

                cv2.imshow("🎯 Target Hit", frame)
                cv2.waitKey(1000)
                cv2.destroyWindow("🎯 Target Hit")

            hit_shown = True

    return matches


def show_live_detections(state: VisionState):
    visCFG = config()  # You may already have this passed instead

    while True:
        with state.frame_lock:
            frame = state.latest_frame.copy() if state.latest_frame is not None else None
        with state.detections_lock:
            detections = list(state.latest_detections)

        if frame is None:
            continue

        # Draw detections
        for x1, y1, x2, y2, conf, class_id in detections:
            if class_id >= len(class_names):
                continue
            label = f"{class_names[class_id]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # ✅ Draw center dot
        cv2.circle(
            frame,
            (visCFG.IMG_CENTER_X, visCFG.IMG_CENTER_Y),
            radius=4,
            color=(255, 0, 0),
            thickness=-1
        )

        cv2.imshow("🍬 All Detections", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



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


def refine_center_by_ellipse(image, bbox, debug=False):
    """
    Refines target center by fitting an ellipse to the masked object region.

    Args:
        image (np.ndarray): Full BGR image.
        bbox (tuple): YOLO box (x1, y1, x2, y2, conf, class_id)
        debug (bool): Show visualization

    Returns:
        (int, int): Refined center coordinates (x, y)
    """
    x1, y1, x2, y2 = map(int, bbox[:4])

    # Shave off pixels to avoid having the wrapper ends distorting the ellipse
    width = x2 - x1
    height = y2 - y1
    aspect_ratio = width / height

    # Shave ratios
    shave_ratio_x = 0.15  # 15% of width
    shave_ratio_y = 0.15  # 15% of height

    # Default trims
    trim_x = int(width * shave_ratio_x)
    trim_y = int(height * shave_ratio_y)

    # Adjust trimming based on aspect ratio
    if aspect_ratio > 1.5:
        # Wider than tall → shave X only
        x1 += trim_x
        x2 -= trim_x
    elif aspect_ratio < 0.67:
        # Taller than wide → shave Y only
        y1 += trim_y
        y2 -= trim_y
    else:
        # Roughly square → shave both
        x1 += trim_x
        x2 -= trim_x
        y1 += trim_y
        y2 -= trim_y

    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(image.shape[1], x2)
    y2 = min(image.shape[0], y2)

    cropped = image[y1:y2, x1:x2].copy()
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)



    # Define masks for red (two ranges), yellow, blue, black, and gold
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    # Red
    mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask, cv2.bitwise_or(mask_red1, mask_red2))

    # Yellow
    mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([35, 255, 255]))
    mask = cv2.bitwise_or(mask, mask_yellow)

    # Blue
    mask_blue = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([130, 255, 255]))
    mask = cv2.bitwise_or(mask, mask_blue)

    # Black (low V)
    mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
    mask = cv2.bitwise_or(mask, mask_black)

    # Gold (muted yellow/orange range)
    mask_gold = cv2.inRange(hsv, np.array([15, 80, 120]), np.array([30, 200, 255]))
    mask = cv2.bitwise_or(mask, mask_gold)

    # Optional: clean up mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if len(largest) >= 5:
            ellipse = cv2.fitEllipse(largest)
            (cx_local, cy_local), (MA, ma), angle = ellipse
            refined_x = x1 + int(cx_local)
            refined_y = y1 + int(cy_local)

            if debug:
                # Convert binary mask to 3-channel grayscale for overlay
                mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

                # Optional: tint the mask (e.g., green channel) for visibility
                tinted_mask = cv2.multiply(mask_rgb, np.array([0, 1, 0], dtype=np.uint8))

                # Blend cropped image with mask
                overlay = cv2.addWeighted(cropped, 0.7, tinted_mask, 0.3, 0)

                # Draw the fitted ellipse on the overlay
                cv2.ellipse(overlay, ellipse, (0, 255, 255), 2)  # yellow ellipse
                cv2.circle(overlay, (int(cx_local), int(cy_local)), 4, (0, 0, 255), -1)  # red centroid

                # Show the final image
                cv2.imshow("Overlay: Cropped Image + Mask + Ellipse", overlay)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            return refined_x, refined_y

    # Fallback: center of bounding box
    return (int((x1 + x2) / 2), int((y1 + y2) / 2))