import cv2
import numpy as np
import glob

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


# Configuration
chessboard_size = (19, 11)      # inner corners (not squares)
square_size = 12.0            # mm per square
image_folder = "test_scripts/Camera_calibration/calibration_images/"  # Folder of captured chessboard images

# Prepare object points (0,0,0), (1,0,0), ...
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object and image points
objpoints = []  # 3D points
imgpoints = []  # 2D image points

images = glob.glob(image_folder + "*.jpg")

for fname in images:
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Calibration', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("✅ Calibration complete")
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# Save parameters to file
np.savez("camera_calibration.npz", camera_matrix=mtx, dist_coeffs=dist)



# TESTING

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)  # or replace with video file or image path

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # Show
    cv2.imshow("Original", frame)
    cv2.imshow("Undistorted", undistorted)

    key = cv2.waitKey(1)
    if key == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()