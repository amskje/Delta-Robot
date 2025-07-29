import os

def delete_img_files(folder_path):
    for filename in os.listdir(folder_path):
        if filename.startswith("IMG"):
            file_path = os.path.join(folder_path, filename)
            if os.path.isfile(file_path):
                print(f"Deleting: {file_path}")
                os.remove(file_path)

# Wipe labelsjekkmappe:
folder1 = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/labelsjekkmappe"
delete_img_files(folder1)

# Wipe unlabeled-images:
folder2 = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/unlabeled-images"
delete_img_files(folder2)

# Wipe outputted-labels:
folder3 = "C:/Users/SkjevrakA/Documents/NOV 2025/DeltaRobot/Train model/code/auto-labeler/outputted-labels"
delete_img_files(folder3)