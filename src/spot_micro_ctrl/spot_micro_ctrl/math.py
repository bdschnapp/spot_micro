# All units are mm or rad unless otherwise stated
import numpy as np

DEBUG = False


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


def _test():
    global DEBUG
    DEBUG = True
    hip, knee = calculate_angles(0.0, -96.0)
    print(hip / np.pi, knee / np.pi)


if __name__ == "__main__":
    _test()
