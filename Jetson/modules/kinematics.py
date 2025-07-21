import math
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class KinematicsConfig:
    # Base geometry [mm]
    r_base: float = 2*70.0
    r_end: float = 2*24.0
    l_biceps: float = 147.0
    l_forearm: float = 250.0

    # Stepper settings
    pulses_per_rev: int = 8000

    # Motion planning
    MAX_waypoints: int = 5

    def __post_init__(self):
        self.tan30 = math.tan(math.radians(30))
        self.cos120 = math.cos(math.radians(120))
        self.sin120 = math.sin(math.radians(120))
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
    conf = config()

    y1 = -0.5 * conf.tan30 * conf.r_base
    y0 -= 0.5 * conf.tan30 * conf.r_end

    a = (x0 * x0 + y0 * y0 + z0 * z0 + conf.l_biceps * conf.l_biceps - conf.l_forearm * conf.l_forearm - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1) * (a + b * y1) + conf.l_biceps * (b * b * conf.l_biceps + conf.l_biceps)
    if d < 0:
        return -1, None

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1)
    zj = a + b * yj
    theta = math.degrees(math.atan(-zj / (y1 - yj)))
    if yj > y1:
        theta += 180.0
    return 0, theta

def inverse_kinematics(x0, y0, z0):
    conf = config()
    status, theta1 = single_arm_ik(x0, y0, z0)
    if status != 0:
        return -1, None, None, None

    status, theta2 = single_arm_ik(x0 * conf.cos120 + y0 * conf.sin120, y0 * conf.cos120 - x0 * conf.sin120, z0)
    if status != 0:
        return -1, None, None, None

    status, theta3 = single_arm_ik(x0 * conf.cos120 - y0 * conf.sin120, y0 * conf.cos120 + x0 * conf.sin120, z0)
    if status != 0:
        return -1, None, None, None

    if theta1 is None or theta2 is None or theta3 is None:
        return 0, theta1, theta2, theta3
    
    # Adjust angles for limit offsets
    return 0, theta1 - conf.ZERO_ANGLE1, theta2 - conf.ZERO_ANGLE2, theta3 - conf.ZERO_ANGLE3

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
        status, theta1, theta2, theta3 = inverse_kinematics(x, y, z)

        # Skip if any angle is clearly invalid (None used as error signal)
        if theta1 == None or theta2 == None or theta3 == None:
            print(f"Skipping invalid angles at step {i}: ({theta1}, {theta2}, {theta3})")
            continue

        angles_list.append((
            round(theta1 * config().pulses_per_rev / 360),
            round(theta2 * config().pulses_per_rev / 360),
            round(theta3 * config().pulses_per_rev / 360)
        ))