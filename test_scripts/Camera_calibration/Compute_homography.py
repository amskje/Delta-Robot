import cv2
import numpy as np

# Parameters
pattern_size = (19, 11)   # 19 inner columns, 11 inner rows
square_size = 12.0        # mm per square

# Load image
img = cv2.imread("chessboard_frame.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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
object_points = np.array([
    [x * square_size, y * square_size]
    for y in range(pattern_size[1])
    for x in range(pattern_size[0])
], dtype=np.float32)

# Compute homography
H, status = cv2.findHomography(corners, object_points)
print("✅ Homography matrix H:\n", H)

np.save("homography_matrix.npy", H)
# Load it later:
# H = np.load("homography_matrix.npy")



# TESTING

def pixel_to_world(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pt, H)
    return world_pt[0][0]  # (x_mm, y_mm)


x_pixel, y_pixel = 312, 421  # From YOLO box center
x_mm, y_mm = pixel_to_world(x_pixel, y_pixel, H)
print(f"Real-world coords: X = {x_mm:.2f} mm, Y = {y_mm:.2f} mm")
