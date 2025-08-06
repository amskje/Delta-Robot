import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ==== CONFIGURATION ====
FRAME_WIDTH = 640
FRAME_HEIGHT = 640
SURFACE_WIDTH_CM = 29.6
SURFACE_HEIGHT_CM = 29.6
CONF_THRESHOLD = 0.7
MODEL_PATH = "modules/weightsV3.pt"  # <== use your .pt file here
# ========================

PIXEL_TO_CM_X = SURFACE_WIDTH_CM / FRAME_WIDTH
PIXEL_TO_CM_Y = SURFACE_HEIGHT_CM / FRAME_HEIGHT
IMG_CENTER_X = FRAME_WIDTH // 2
IMG_CENTER_Y = FRAME_HEIGHT // 2

class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "Karamell", "Lakris", "Notti", "Toffee", "Eclairs", "Marsipan"
]

# === Load YOLOv8 Model with GPU ===
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = YOLO(MODEL_PATH)
print(f"[INFO] Model loaded on device: {device}")

# === GStreamer Camera Pipeline ===
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("🚫 Failed to open camera.")
    exit()

cv2.namedWindow("🍬 YOLOv8 PyTorch Live", cv2.WINDOW_NORMAL)
cv2.resizeWindow("🍬 YOLOv8 PyTorch Live", FRAME_WIDTH, FRAME_HEIGHT)

print("🎥 PyTorch live detection running... Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to read frame.")
        break

    # Run YOLOv8 Inference
    results = model(frame, device=device.index, verbose=False)[0]

    boxes_tensor = results.boxes.xyxy.cpu().numpy()
    confs_tensor = results.boxes.conf.cpu().numpy()
    class_ids_tensor = results.boxes.cls.cpu().numpy()

    detections = []
    for i in range(len(boxes_tensor)):
        x1, y1, x2, y2 = boxes_tensor[i].astype(int)
        conf = float(confs_tensor[i])
        class_id = int(class_ids_tensor[i])
        if conf > CONF_THRESHOLD:
            detections.append((x1, y1, x2, y2, conf, class_id))

    if not detections:
        print("⚠️ No detections this frame.")

    for x1, y1, x2, y2, conf, class_id in detections:
        label = f"{class_names[class_id]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        x_pixel = int((x1 + x2) / 2)
        y_pixel = int((y1 + y2) / 2)
        x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
        y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

        coord_label = f"({x_cm:.1f}, {y_cm:.1f}) cm"
        cv2.putText(frame, coord_label, (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.imshow("🍬 YOLOv8 PyTorch Live", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 Exiting...")
        break

cap.release()
cv2.destroyAllWindows()
