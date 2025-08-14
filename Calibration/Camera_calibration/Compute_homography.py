import cv2
import numpy as np

# Parameters
pattern_size = (11, 19)   # 19 inner columns, 11 inner rows
square_size = 11.95       # mm per square

camera_cal = np.load("camera_calibration.npz")
mtx = camera_cal['camera_matrix']
dist = camera_cal['dist_coeffs']

# Load image
img = cv2.imread("test_scripts/Camera_calibration/calibration_images/chessboard_frame.jpg")
if img is None:
    print("❌ Image not loaded. Check the file path.")
    exit()

# ✅ UNDISTORT the image
undistorted = cv2.undistort(img, mtx, dist)

# Convert to grayscale
gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

# Find chessboard corners
ret, corners = cv2.findChessboardCorners(gray, pattern_size)

if not ret:
    print("❌ Could not find chessboard corners.")
    exit()

# Refine corner positions
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

# Draw detected corners
cv2.drawChessboardCorners(img, pattern_size, corners, ret)
cv2.imshow("Detected Corners", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Generate world coordinates
board_width = pattern_size[0] * square_size
board_height = pattern_size[1] * square_size

object_points = np.array([
    [-(y * square_size - board_height / 2),  # X axis: upward in image
     x * square_size - board_width / 2]     # Y axis: rightward in image
    for y in range(pattern_size[1])
    for x in range(pattern_size[0])
], dtype=np.float32)


# Compute homography
H, status = cv2.findHomography(corners, object_points)
print("✅ Homography matrix H:\n", H)

np.save("homography_camera.npy", H)
# Load it later:
# H = np.load("homography_matrix.npy")



# TESTING

def pixel_to_world(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pt, H)
    return world_pt[0][0]  # (x_mm, y_mm)


x_pixel, y_pixel = 600, 320  # From YOLO box center
x_mm, y_mm = pixel_to_world(x_pixel, y_pixel, H)
print(f"Real-world coords: X = {x_mm:.2f} mm, Y = {y_mm:.2f} mm")