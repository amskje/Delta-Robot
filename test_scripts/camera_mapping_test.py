import cv2
import numpy as np

# Load image and homography
img = cv2.imread("test_scripts/Camera_calibration/calibration_images/chessboard_frame.jpg")
camera_cal = np.load("camera_calibration.npz")
mtx = camera_cal['camera_matrix']
dist = camera_cal['dist_coeffs']
H = np.load("homography_camera.npy")

# Undistort image
undistorted = cv2.undistort(img, mtx, dist)

# === Define world coordinates to test (mm) ===
# Format: (X_mm (up), Y_mm (right))
test_world_points = [
    (0, 0),             # center
    (59.75, 0),         # 60mm up
    (-59.75, 0),        # 60mm down
    (0, 59.75),         # 60mm right
    (0, -59.75),        # 60mm left
    (107.55, 71.7),     # near a corner
]

# Apply world coordinate offset correction (based on observed shift)
# Shifting world coords by +6 mm in X and -6 mm in Y
offset_correction = np.array([6.0, -6.0], dtype=np.float32)
corrected_world_points = [np.array(pt) + offset_correction for pt in test_world_points]

# Convert to (N,1,2) shape for cv2.perspectiveTransform
world_pts = np.array(corrected_world_points, dtype=np.float32).reshape(-1, 1, 2)

# Project to image using inverse homography
image_pts = cv2.perspectiveTransform(world_pts, np.linalg.inv(H))

# Draw smaller dots and labels
for pt, label in zip(image_pts, test_world_points):  # label with original coords
    u, v = int(pt[0][0]), int(pt[0][1])
    cv2.circle(undistorted, (u, v), 3, (0, 0, 255), -1)
    cv2.putText(undistorted, f"{label}", (u + 5, v - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

# Show image
cv2.imshow("Projected World Points (with offset correction)", undistorted)
cv2.waitKey(0)
cv2.destroyAllWindows()
