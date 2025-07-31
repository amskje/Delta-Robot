import cv2
from ultralytics import YOLO


# ==== CONFIGURATION ====
FRAME_WIDTH = 640         # width of video feed in pixels
FRAME_HEIGHT = 640        # height of video feed in pixels
SURFACE_WIDTH_CM = 29.6   # physical surface width in cm
SURFACE_HEIGHT_CM = 29.6  # physical surface height in cm
CONF_THRESHOLD = 0.75      # minimum confidence for detection
MODEL_PATH = "modules/weightsV2.pt"
# ========================

# Derived conversion factors
PIXEL_TO_CM_X = SURFACE_WIDTH_CM / FRAME_WIDTH
PIXEL_TO_CM_Y = SURFACE_HEIGHT_CM / FRAME_HEIGHT
IMG_CENTER_X = FRAME_WIDTH // 2
IMG_CENTER_Y = FRAME_HEIGHT // 2

# Custom class names
class_names = [
    "Banan", "Cocos", "Crisp", "Daim", "Fransk", "Golden",
    "Japp", "Karamell", "Lakris", "Notti", "Toffee", "Eclairs", "Marsipan"
]

# Load YOLO model
model = YOLO(MODEL_PATH)

# GStreamer pipeline with center-crop to square (2464x2464) and downscale to 640x640
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv left=408 top=0 right=2872 bottom=2464 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Initialize camera
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("🚫 Failed to open camera.")
    exit()

# Create square display window
cv2.namedWindow("🍬 YOLOv8 Live Detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("🍬 YOLOv8 Live Detection", FRAME_WIDTH, FRAME_HEIGHT)

print("🎥 Live detection running... Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to read frame.")
        break

    # Run YOLO inference
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(frame_rgb, conf=CONF_THRESHOLD)[0]
    annotated_frame = results.plot()

    # Loop over detections
    for box in results.boxes:
        x_pixel = int(box.xywh[0][0].item())
        y_pixel = int(box.xywh[0][1].item())

        # Draw red center dot
        cv2.circle(annotated_frame, (x_pixel, y_pixel), radius=5, color=(0, 0, 255), thickness=-1)

        # Convert to real-world centered coordinates
        x_cm = (x_pixel - IMG_CENTER_X) * PIXEL_TO_CM_X
        y_cm = (y_pixel - IMG_CENTER_Y) * PIXEL_TO_CM_Y

        # Overlay coordinate text
        coord_label = f"({x_cm:.1f}, {y_cm:.1f}) cm"
        cv2.putText(annotated_frame, coord_label, (x_pixel + 5, y_pixel - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show frame
    cv2.imshow("🍬 YOLOv8 Live Detection", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 Exiting...")
        break

cap.release()
cv2.destroyAllWindows()
