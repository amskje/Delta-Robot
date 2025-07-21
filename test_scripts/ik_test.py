import math
import kinematics
import kinematics_new

# Robot geometry
# (look at pics above for explanation)
e = 48.0     # end effector
f = 140.0     # base
re = 250.0
rf = 147.0

# Trigonometric constants
sqrt3 = math.sqrt(3.0)
pi = math.pi
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

def delta_calcForward(theta1, theta2, theta3):
    t = (f - e) * tan30 / 2
    dtr = pi / 180.0

    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    y1 = -(t + rf * math.cos(theta1))
    z1 = -rf * math.sin(theta1)

    y2 = (t + rf * math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)

    y3 = (t + rf * math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm)**2 + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

    d = b * b - 4.0 * a * c
    if d < 0:
        return -1, None, None, None

    z0 = -0.5 * (b + math.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm

    return 0, x0, y0, z0

def delta_calcAngleYZ(x0, y0, z0):
    y1 = -0.5 * 0.57735 * f
    y0 -= 0.5 * 0.57735 * e

    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if d < 0:
        return -1, None

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1)
    zj = a + b * yj
    theta = math.degrees(math.atan(-zj / (y1 - yj)))
    if yj > y1:
        theta += 180.0
    return 0, theta

def delta_calcInverse(x0, y0, z0):
    status, theta1 = delta_calcAngleYZ(x0, y0, z0)
    if status != 0:
        return -1, None, None, None

    status, theta2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0)
    if status != 0:
        return -1, None, None, None

    status, theta3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0)
    if status != 0:
        return -1, None, None, None

    return 0, theta1, theta2, theta3


def main():
    # Example usage
    x0, y0, z0 = 20.0, 30.0, -290.0  # Example coordinates
    status, theta1, theta2, theta3 = delta_calcInverse(x0, y0, z0)
    
    if status == 0:
        print(f"Inverse Kinematics Results: Theta1: {theta1}, Theta2: {theta2}, Theta3: {theta3}")
    else:
        print("Inverse Kinematics failed.")

    theta1, theta2, theta3 = kinematics.inverse_kinematics(x0, y0, z0)
    print(f"Kinematics Results: Theta1: {theta1}, Theta2: {theta2}, Theta3: {theta3}")

    status, theta1, theta2, theta3 = kinematics_new.inverse_kinematics(x0, y0, z0)
    print(f"Kinematics New Results: Theta1: {theta1}, Theta2: {theta2}, Theta3: {theta3}")

    #theta1, theta2, theta3 = 22.86, 22.095, 23.49

    # Forward kinematics example
    status, x0, y0, z0 = delta_calcForward(theta1, theta2, theta3)
    if status == 0:
        print(f"Forward Kinematics Results: X: {x0}, Y: {y0}, Z: {z0}") 


if __name__ == "__main__":
    main()