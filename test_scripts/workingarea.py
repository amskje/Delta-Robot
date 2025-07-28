import numpy as np
import kinematics_new
import matplotlib.pyplot as plt

# Settings
z_values = np.arange(-285, -335, -1)
xy_range = np.arange(-250, 251, 5)
grid_resolution = 5  # mm
angle_limit_low =18.0  # Degrees (minimum "downward" angle)
angle_limit_high = 90 + 18.2

max_area = 0
best_z = None

for z in z_values:
    valid_count = 0
    for x in xy_range:
        for y in xy_range:
            status, t1, t2, t3 = kinematics_new.inverse_kinematics(x, y, z)
            if status != 0:
                continue
            # Apply joint angle constraint
            if t1 is None or t2 is None or t3 is None:
                continue
            if min(t1, t2, t3) < angle_limit_low:
                continue
            if max(t1, t2, t3) > angle_limit_high:
                continue
            valid_count += 1
    area = valid_count * (grid_resolution ** 2)
    print(f"Z = {z} mm -> Reachable area = {area / 100.0:.2f} cm²")
    if area > max_area:
        max_area = area
        best_z = z

print(f"\n✅ Max reachable XY area (with joint limits) is at Z = {best_z} mm")
print(f"   Area = {max_area:.2f} mm² ({max_area / 100.0:.2f} cm²)")


reachable_x = []
reachable_y = []

for x in xy_range:
    for y in xy_range:
        status, t1, t2, t3 = kinematics_new.inverse_kinematics(x, y, best_z)  # Use the best Z value found
        if status != 0:
            continue
        if t1 is None or t2 is None or t3 is None:
            continue
        if min(t1, t2, t3) < angle_limit_low:
            print(f"Rejected ({x}, {y}) at Z={best_z}: angle too low (angles: {t1:.2f}, {t2:.2f}, {t3:.2f})")
            continue
        if max(t1, t2, t3) > angle_limit_high:
            print(f"Rejected ({x}, {y}) at Z={best_z}: angle too high (angles: {t1:.2f}, {t2:.2f}, {t3:.2f})")
            continue
        reachable_x.append(x)
        reachable_y.append(y)

# Plot the XY working area
plt.figure(figsize=(6, 6))
plt.scatter(reachable_x, reachable_y, s=5, color='blue')
plt.title(f"XY Working Area at Z = {best_z} mm (with joint limit ≥ 18° down)")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.show()