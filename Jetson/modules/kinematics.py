import math
from typing import List, Tuple

# Constants
tan30 = math.tan(math.radians(30))
r_base = 70.0
r_end = 24.0
l_biceps = 147.0
l_forearm = 250.0
pulses_per_rev = 800

ZERO_ANGLE1 = 408*360 / pulses_per_rev
ZERO_ANGLE2 = 391*360 / pulses_per_rev
ZERO_ANGLE3 = 422*360 / pulses_per_rev

MAX_waypoints = 10

path_points = []
path_angles = []
path_index = 0
path_running = False

class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

def single_arm_ik(x0, y0, z0):
    """Computes the angle for one arm given x, y, z.
    Returns (success, thetaDeg)"""
    y1 = -0.5 * tan30 * r_base
    y0 -= 0.5 * tan30 * r_end

    a = (x0**2 + y0**2 + z0**2 + l_biceps**2 - l_forearm**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1)**2 + l_biceps * (b**2 * l_biceps + l_biceps)

    print(f"  [IK] x={x0:.2f}, y={y0:.2f}, z={z0:.2f}, d={d:.2f}")

    if d < 0:
        print(f"  → FAIL: d={d:.2f} < 0 → unreachable")
        return False, 0.0


    yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
    zj = a + b * yj

    theta = math.atan2(-zj, y1 - yj) * 180.0 / math.pi
    return True, theta

def rotate_xy(x, y, angle_rad):
    xr = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    yr = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return xr, yr

def inverse_kinematics(x, y, z):
    """Returns tuple (theta1, theta2, theta3) in degrees.
    If any arm fails, it returns 0.0 for that angle."""
    # Arm 1
    ok1, t1 = single_arm_ik(x, y, z)
    theta1 = t1 - ZERO_ANGLE1 if ok1 else 0.0

    # Arm 2
    xr2, yr2 = rotate_xy(x, y, 2.0 * math.pi / 3.0)
    ok2, t2 = single_arm_ik(xr2, yr2, z)
    theta2 = t2 - ZERO_ANGLE2 if ok2 else 0.0

    # Arm 3
    xr3, yr3 = rotate_xy(x, y, -2.0 * math.pi / 3.0)
    ok3, t3 = single_arm_ik(xr3, yr3, z)
    theta3 = t3 - ZERO_ANGLE3 if ok3 else 0.0

    if not ok1: print("  → Arm 1 failed")
    if not ok2: print("  → Arm 2 failed")
    if not ok3: print("  → Arm 3 failed")

    return theta1, theta2, theta3

def plan_linear_move(
    x0: float, y0: float, z0: float,
    x1: float, y1: float, z1: float,
    angles_list: List[Tuple[int, int, int]],
    waypoints: int = MAX_waypoints
    ):
    """
    Generate linear path from (x0, y0, z0) to (x1, y1, z1), compute IK for each step,
    and fill the given `angles_list` with tuples of (theta1, theta2, theta3).

    Parameters:
        x0, y0, z0: Start coordinates
        x1, y1, z1: End coordinates
        angles_list: List to be filled with IK angle tuples
        waypoints: Number of linear interpolation steps (default 100)
    """
    if waypoints > MAX_waypoints:
        waypoints = MAX_waypoints

    angles_list.clear()  # Ensure the list is empty before filling

    for i in range(waypoints):
        t = i / (waypoints - 1)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)

        theta1, theta2, theta3 = inverse_kinematics(x, y, z)

        # Skip if any angle is clearly invalid (0.0 used as error signal)
        if theta1 == 0.0 or theta2 == 0.0 or theta3 == 0.0:
            print(f"  → Skipping invalid IK at x={x:.2f}, y={y:.2f}, z={z:.2f}")
            continue

        angles_list.append((
            round(theta1 * pulses_per_rev / 360),
            round(theta2 * pulses_per_rev / 360),
            round(theta3 * pulses_per_rev / 360)
        ))
