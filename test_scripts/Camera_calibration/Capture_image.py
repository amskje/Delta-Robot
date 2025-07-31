import cv2


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

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)  # or replace with video file or image path

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Live", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):  # Press 's' to save a frame for calibration
        cv2.imwrite("test_scripts/Camera_calibration/calibration_images/chessboard_frame.jpg", frame)
        print("Saved chessboard image")
        break

cap.release()
cv2.destroyAllWindows()
