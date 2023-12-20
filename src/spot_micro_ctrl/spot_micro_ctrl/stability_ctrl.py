from spot_micro_ctrl.math import XYRobotLegState, quaternion_to_euler, PID


class HeightController:
    def __init__(self, rosparams=None):
        self.xy_robot_leg_state = XYRobotLegState(rosparams)


class StabilityController:
    def __init__(self, rosparams=None):
        self.roll = 0
        self.pitch = 0
        self.kp = 1
        self.xy_robot_leg_state = None
        self.r_pid = PID(1, 0, 0) 
        self.p_pid = PID(1, 0, 0)

    # callback for /spot_micro/gpio/imu topic set in SpotMicroCtrl node
    def imu_sub(self, msg):
        euler = quaternion_to_euler(msg.data.orientation)
        self.roll = euler[0]
        self.pitch = euler[1]

    def run_10ms(self):
        p = self.p_pid(self.pitch)
        r = self.r_pid(self.roll)
        xy = XYRobotLegState()
        xy.zero()
        xy.add_front(0, p / 2)
        xy.add_back(0, -1 * p / 2)
        xy.add_left(0, -1 * r / 2)
        xy.add_right(0, r / 2)

        self.xy_robot_leg_state = xy

    def get_xy(self):
        return self.xy_robot_leg_state
