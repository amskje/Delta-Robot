import os
from collections import Counter

def count_yolo_classes(labels_dir):
    class_counts = Counter()
    for txt_file in os.listdir(labels_dir):
        if txt_file.endswith('.txt'):
            with open(os.path.join(labels_dir, txt_file), 'r') as f:
                for line in f:
                    if line.strip():
                        class_id = int(line.split()[0])
                        class_counts[class_id] += 1
    return class_counts

# Usage with your path
labels_path = r'C:\Users\SkjevrakA\Documents\NOV 2025\DeltaRobot\Train model\code\model5\labels\train'
counts = count_yolo_classes(labels_path)

# Sort by class_id for easier reading
for class_id in sorted(counts.keys()):
    print(f"Class {class_id}: {counts[class_id]} instances")

print(f"\nTotal instances: {sum(counts.values())}")