import math

# Constants
tan30 = math.tan(math.radians(30))
r_base = 70.0
r_end = 24.0
l_biceps = 147.0
l_forearm = 250.0

MAX_waypoints = 20

path_points = []
path_angles = []
path_index = 0
path_running = False

def single_arm_ik(x0, y0, z0):
    """Computes the angle for one arm given x, y, z.
    Returns (success, thetaDeg)"""
    y1 = -0.5 * tan30 * r_base
    y0 -= 0.5 * tan30 * r_end

    a = (x0**2 + y0**2 + z0**2 + l_biceps**2 - l_forearm**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1)**2 + l_biceps * (b**2 * l_biceps + l_biceps)

    if d < 0:
        return False, 0.0  # Not reachable

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
    theta1 = t1 if ok1 else 0.0

    # Arm 2
    xr2, yr2 = rotate_xy(x, y, 2.0 * math.pi / 3.0)
    ok2, t2 = single_arm_ik(xr2, yr2, z)
    theta2 = t2 if ok2 else 0.0

    # Arm 3
    xr3, yr3 = rotate_xy(x, y, -2.0 * math.pi / 3.0)
    ok3, t3 = single_arm_ik(xr3, yr3, z)
    theta3 = t3 if ok3 else 0.0

    if not (ok1 and ok2 and ok3):
        print("IK solution failed for one or more arms.")

    return theta1, theta2, theta3


def plan_linear_move(x0, y0, z0, x1, y1, z1, angles_list, waypoints=20):
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
        angles_list.append((theta1, theta2, theta3))
