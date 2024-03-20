# TODO: remove 
# All units are mm or rad unless otherwise stated
import numpy as np
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

DEBUG = False


def quaternion_to_euler(quat: Quaternion):
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return [roll, pitch, yaw]


def law_of_cosine(a: float, b: float, c: float) -> float:
    # returns the angle across from length c
    if DEBUG:
        print(a, b, c)
        print("arccos", np.arccos((np.square(a) + np.square(b) - np.square(c)) / np.abs(2 * a * b)))
    return np.arccos((np.square(a) + np.square(b) - np.square(c)) / np.abs(2 * a * b))


def calculate_angles(x: float, y: float, a=68, c=68):
    b = np.sqrt(np.square(x) + np.square(y))
    knee_servo_target = np.pi - law_of_cosine(a, c, b)

    C = law_of_cosine(a, b, c)
    offset = np.arcsin(x / b) + ((np.pi/2) * (x <= 0))
    hip_servo_target = np.pi - (C + offset)
    return hip_servo_target, knee_servo_target


def calculate_xy(hip_angle, knee_angle, a=68, c=68):
    pass


class XYRobotLegState:
    def __init__(self, rosparams=None):
        self.points = [0, -96.16, 0, -96.16, 0, -96.16, 0, -96.16]
        if rosparams:
            # initialize points with rosparams
            pass

    def __add__(self, other):
        if type(other) != XYRobotLegState:
            raise Exception(TypeError, "Unable to add XYRobotLegState with ", str(type(other)))
        new_state = XYRobotLegState()
        for _, index in enumerate(self.points):
            new_state.points[index] = self.points[index] + other.points[index]
        return new_state

    def calculate_angles(self):
        angles = []
        for i in range(4):
            x = self.points[2 * i]
            y = self.points[(2 * i) + 1]
            angles.extend([calculate_angles(x, y)])
        return str(angles).lstrip('[').rstrip(']')

    def zero(self):
        self.points = [0, 0, 0, 0, 0, 0, 0, 0]

    def add_right(self, x, y):
        self.points[0] += x
        self.points[1] += y
        self.points[2] += x
        self.points[3] += y

    def add_left(self, x, y):
        self.points[4] += x
        self.points[5] += y
        self.points[6] += x
        self.points[7] += y

    def add_front(self, x, y):
        self.points[0] += x
        self.points[1] += y
        self.points[6] += x
        self.points[7] += y

    def add_back(self, x, y):
        self.points[2] += x
        self.points[3] += y
        self.points[4] += x
        self.points[5] += y


class PID:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.i = 0
        self.e_prev = 0
        self.dt = dt

    def update(self, measurement, sp=0):
        e = sp - measurement
        p = self.kp * e
        self.i += self.ki * e * self.dt
        d = self.kd * (e - self.e_prev) / self.dt
        self.e_prev = e
        return p + self.i + d
