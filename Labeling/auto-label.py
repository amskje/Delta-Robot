import os
import shutil
from ultralytics import YOLO

# === CONFIGURATION ===
model_path = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/runs/detect/train50/weights/best50.pt"
input_dir = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/unlabeled-images"
output_dir = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/outputted-labels"
review_dir = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/labelsjekkmappe"
confidence_threshold = 0.5

# === INIT YOLO MODEL ===
model = YOLO(model_path)

# === PREPARE OUTPUT FOLDERS ===
os.makedirs(output_dir, exist_ok=True)
os.makedirs(review_dir, exist_ok=True)

# === GET IMAGE LIST ===
image_extensions = (".jpg", ".jpeg", ".png", ".bmp")
image_files = [f for f in os.listdir(input_dir) if f.lower().endswith(image_extensions)]

print(f"Found {len(image_files)} images to label...")

# === RUN INFERENCE AND SAVE LABEL FILES ===
for i, filename in enumerate(image_files):
    image_path = os.path.join(input_dir, filename)
    image_stem = os.path.splitext(filename)[0]
    label_path = os.path.join(output_dir, f"{image_stem}.txt")

    # Run YOLO prediction
    results = model.predict(image_path, conf=confidence_threshold, save=False)
    result = results[0]
    boxes = result.boxes

    # Always create label file (even if empty)
    with open(label_path, "w") as f:
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                class_id = int(box.cls[0].item())
                x_center, y_center, width, height = box.xywhn[0]
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

    print(f"[{i+1}/{len(image_files)}] Saved labels for: {filename} with {len(boxes) if boxes else 0} boxes")

# === COPY IMAGES AND LABELS TO REVIEW FOLDER ===
for filename in os.listdir(output_dir):
    if filename.endswith(".txt"):
        image_name = filename.replace(".txt", ".JPG")  # adjust extension if needed
        image_path = os.path.join(input_dir, image_name)
        label_path = os.path.join(output_dir, filename)

        if os.path.exists(image_path):
            shutil.copy(image_path, review_dir)
        shutil.copy(label_path, review_dir)

print("\n✅ Done. Review folder populated at:", review_dir)
print(f'👾👾👾 Open labelImg like this to validate labels: labelImg "{review_dir}" "{os.path.join(review_dir, "classes.txt")}"')
