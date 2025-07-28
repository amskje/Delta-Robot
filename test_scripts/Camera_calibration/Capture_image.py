import cv2

cap = cv2.VideoCapture(0)  # or replace with video file or image path

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Live", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):  # Press 's' to save a frame for calibration
        cv2.imwrite("chessboard_frame.jpg", frame)
        print("Saved chessboard image")
        break

cap.release()
cv2.destroyAllWindows()
