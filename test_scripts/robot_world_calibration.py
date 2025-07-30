import numpy as np
import cv2
import json

# ---- Step 1: Define your calibration pairs ----

# Points you *commanded* the robot to go to
robot_coords = np.array([
    [-100.0, -100.0],
    [ -50.0, -100.0],
    [   0.0, -100.0],
    [  50.0, -100.0],
    [ 100.0, -100.0],
    [-100.0,  -50.0],
    [ -50.0,  -50.0],
    [   0.0,  -50.0],
    [  50.0,  -50.0],
    [ 100.0,  -50.0],
    [-100.0,    0.0],
    [ -50.0,    0.0],
    [   0.0,    0.0],
    [  50.0,    0.0],
    [ 100.0,    0.0],
    [-100.0,   50.0],
    [ -50.0,   50.0],
    [   0.0,   50.0],
    [  50.0,   50.0],
    [ 100.0,   50.0],
    [-100.0,  100.0],
    [ -50.0,  100.0],
    [   0.0,  100.0],
    [  50.0,  100.0],
    [ 100.0,  100.0]
], dtype=np.float32)


# Where the robot *actually went*, measured on your printed grid
real_coords = np.array([
    [-92, -90],
    [-50, -90],
    [ -7, -91],
    [ 37, -91],
    [ 80, -92],
    [-93, -44],
    [-48, -45],
    [ -3, -45],
    [ 40, -46],
    [ 84, -45],
    [-93,   1],
    [-47,   1],
    [ -3,   0],
    [ 41,  -2],
    [ 86,   0],
    [-93,  46],
    [-47,  45],
    [ -2,  44],
    [ 43,  44],
    [ 89,  44],
    [-94,  89],
    [-47,  89],
    [ -1,  88],
    [ 45,  88],
    [ 92,  88]
], dtype=np.float32)


# ---- Step 2: Calculate the homography from robot → real ----

H, status = cv2.findHomography(robot_coords, real_coords)

# Optional: save homography to file
with open("homography_ROBOT_WORLD.json", "w") as f:
    json.dump(H.tolist(), f)

print("Homography matrix:\n", H)

# ---- Step 3: Apply correction to new commands ----

def correct_target(x_desired, y_desired, H_inv):
    """Transform a target point from real-world into robot coordinates"""
    pt = np.array([[[x_desired, y_desired]]], dtype=np.float32)
    corrected = cv2.perspectiveTransform(pt, H_inv)
    return corrected[0][0]

# Invert matrix: real → robot
H_inv = np.linalg.inv(H)

# Example: you want to move to point (75, 25) in real-world space
x_real, y_real = 100, 100
x_robot, y_robot = correct_target(x_real, y_real, H_inv)

print(f"Move robot to: ({x_robot:.2f}, {y_robot:.2f}) to reach ({x_real}, {y_real})")
