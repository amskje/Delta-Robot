import cv2
from ultralytics import YOLO
from dataclasses import dataclass, field
import numpy as np
import threading

import time


import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import cv2
import torch

from ultralytics import YOLO
import torch


latest_detections = []
detections_lock = threading.Lock()

__all__ = ["latest_frame"] 


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
    "Japp", "Karamell", "Lakris", "Notti", "Toffee", "Eclairs", "Marsipan"
]

latest_frame = None
frame_lock = threading.Lock()

# === FUNCTIONS ===

def start_inference_thread(model, config):
    global latest_frame, latest_detections

    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[Vision] ❌ GStreamer camera failed to open")
        return
    else:
        print("[Vision] ✅ GStreamer camera opened successfully")


    def inference_loop():
        global latest_frame 
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Failed to read frame.")
                continue
            #else:
             #   print("[Vision] ✅ Grabbed a frame")

            with frame_lock:
                latest_frame = frame.copy()
              #  print("[Vision] ✅ Updated latest_frame")

            # Undistort
            h, w = frame.shape[:2]
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(config.mtx, config.dist, (w, h), 1, (w, h))
            #undistorted = cv2.undistort(frame, config.mtx, config.dist, None, newcameramtx)

            # Run inference
            print(f"[DEBUG] Running inference...")

            
            #results = model(undistorted, device=0, verbose=False)[0]
            results = model(frame, device=device)[0]
            
            boxes_tensor = results.boxes.xyxy.cpu().numpy()
            confs_tensor = results.boxes.conf.cpu().numpy()
            class_ids_tensor = results.boxes.cls.cpu().numpy()

            #print(f"[DEBUG] Found {len(boxes_tensor)} raw detections")

            detections = []
            for i in range(len(boxes_tensor)):
                x1, y1, x2, y2 = boxes_tensor[i].astype(int)
                conf = float(confs_tensor[i])
                class_id = int(class_ids_tensor[i])
                if conf > config.CONF_THRESHOLD:
                    detections.append((x1, y1, x2, y2, conf, class_id))
            
            print(f"[DEBUG] Final detections above threshold: {len(detections)}")
           

            #print(f"[Vision] ✅ Inference ran. Found {len(detections)} objects")

            with detections_lock:
                latest_detections.clear()
                latest_detections.extend(detections)

            #print("[DEBUG] latest_detections updated:", detections)


            for x1, y1, x2, y2, conf, class_id in detections:
                if class_id >= len(class_names):
                    print(f"[Vision] ⚠️ Skipping invalid class_id: {class_id}")
                    continue
                """
                label = f"{class_names[class_id]} {conf:.2f}"
                cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(undistorted, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                """


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

def detect_target(target_class, H):
    with detections_lock:
        detections = list(latest_detections)
        print("[DEBUG detect_target] latest_detections id:", id(latest_detections))

    matches = []
    hit_shown = False

    print("[DETECT TARGET] Got detections:", detections)


    for x1, y1, x2, y2, conf, class_id in detections:
        class_name = class_names[class_id]
        if class_name != target_class:
            continue

        # Compute center
        x_pixel = int((x1 + x2) / 2)
        y_pixel = int((y1 + y2) / 2)
        x_mm, y_mm = pixel_to_world(x_pixel, y_pixel, H)

        matches.append((class_name, x_mm, y_mm, conf))

        print(f"[DETECT TARGET] Checking {class_name} vs {target_class}")


        # Show current frame with hit visualization (use latest at hit moment)
        if not hit_shown:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None

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


def show_live_detections():
    while True:
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
        with detections_lock:
            detections = list(latest_detections)

        print("[LIVE DEBUG] Detections to draw:", detections)


        if frame is None:
            continue

        for x1, y1, x2, y2, conf, class_id in detections:
            if class_id >= len(class_names):
                continue
            label = f"{class_names[class_id]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        draw_overlay(frame)
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