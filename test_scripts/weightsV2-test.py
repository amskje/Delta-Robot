import cv2
from ultralytics import YOLO

model = YOLO("Jetson/modules/weightsV2.pt")  # Make sure it's the right file
image = cv2.imread("test_scripts/YOLO_pictures3/YOLO036.jpg")

results = model(image)[0]
print(f"Detected {len(results.boxes)} boxes")

annotated = results.plot()
cv2.imshow("Result", annotated)
cv2.waitKey(ord('q'))
cv2.destroyAllWindows()
