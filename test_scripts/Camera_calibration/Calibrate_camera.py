import cv2
import numpy as np
import glob

# Configuration
chessboard_size = (9, 6)      # inner corners (not squares)
square_size = 12.0            # mm per square
image_folder = "calibration_images/"  # Folder of captured chessboard images

# Prepare object points (0,0,0), (1,0,0), ...
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object and image points
objpoints = []  # 3D points
imgpoints = []  # 2D image points

images = glob.glob(image_folder + "*.jpg")

for fname in images:
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

cap = cv2.VideoCapture(0)  # or use video file

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