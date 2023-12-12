from spot_micro_ctrl.math import XYRobotLegState

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, Point


class StabilityController:
    def __init__(self, rosparams=None):
        self.imu = Imu
        self.xy_robot_leg_state = XYRobotLegState(rosparams)

    # callback for /spot_micro/gpio/imu topic set in SpotMicroCtrl node
    def imu_sub(self, msg):
        self.imu = msg.data

    def run_10ms(self):
        pass

    def get_xy(self):
        return self.xy_robot_leg_state
