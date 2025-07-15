import math
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class KinematicsConfig:
    # Base geometry [mm]
    r_base: float = 70.0
    r_end: float = 24.0
    l_biceps: float = 147.0
    l_forearm: float = 250.0

    # Stepper settings
    pulses_per_rev: int = 80000

    # Motion planning
    MAX_waypoints: int = 10

    # Derived/calculated constants
    tan30: float = None
    ZERO_ANGLE1: float = None
    ZERO_ANGLE2: float = None
    ZERO_ANGLE3: float = None

    def __post_init__(self):
        self.tan30 = math.tan(math.radians(30))
        self.ZERO_ANGLE1 = 408 * 360 / self.pulses_per_rev
        self.ZERO_ANGLE2 = 391 * 360 / self.pulses_per_rev
        self.ZERO_ANGLE3 = 422 * 360 / self.pulses_per_rev

def config() -> KinematicsConfig:
    return KinematicsConfig()

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
    y1 = -0.5 * config().tan30 * config().r_base
    y0 -= 0.5 * config().tan30 * config().r_end

    a = (x0**2 + y0**2 + z0**2 + config().l_biceps**2 - config().l_forearm**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1)**2 + config().l_biceps * (b**2 * config().l_biceps + config().l_biceps)

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
    theta1 = t1 - config().ZERO_ANGLE1 if ok1 else 0.0

    # Arm 2
    xr2, yr2 = rotate_xy(x, y, 2.0 * math.pi / 3.0)
    ok2, t2 = single_arm_ik(xr2, yr2, z)
    theta2 = t2 - config().ZERO_ANGLE2 if ok2 else 0.0

    # Arm 3
    xr3, yr3 = rotate_xy(x, y, -2.0 * math.pi / 3.0)
    ok3, t3 = single_arm_ik(xr3, yr3, z)
    theta3 = t3 - config().ZERO_ANGLE3 if ok3 else 0.0

    if not ok1: print("  → Arm 1 failed")
    if not ok2: print("  → Arm 2 failed")
    if not ok3: print("  → Arm 3 failed")

    return theta1, theta2, theta3

def plan_linear_move(
    x0: float, y0: float, z0: float,
    x1: float, y1: float, z1: float,
    angles_list: List[Tuple[int, int, int]],
    waypoints: int
    ):
    """
    Generate linear path from (x0, y0, z0) to (x1, y1, z1), compute IK for each step,
    and fill the given `angles_list` with tuples of (theta1, theta2, theta3).

    Parameters:
        x0, y0, z0: Start coordinates
        x1, y1, z1: End coordinates
        angles_list: List to be filled with IK angle tuples
        waypoints: Number of linear interpolation steps
    """

    angles_list.clear()  # Ensure the list is empty before filling

    for i in range(waypoints):
        t = i / (waypoints - 1)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)

        theta1, theta2, theta3 = inverse_kinematics(x, y, z)

        # Skip if any angle is clearly invalid (0.0 used as error signal)
        if theta1 == 0.0 or theta2 == 0.0 or theta3 == 0.0:
            continue

        angles_list.append((
            round(theta1 * config().pulses_per_rev / 360),
            round(theta2 * config().pulses_per_rev / 360),
            round(theta3 * config().pulses_per_rev / 360)
        ))
