from spot_micro_ctrl.math import XYRobotLegState

from geometry_msgs.msg import Twist, Vector3


class TrajectoryGenerator:
    def __init__(self, rosparams=None):
        self.target_velocity = Twist()
        self.robot_leg_state = XYRobotLegState(rosparams)

    # callback for /spot_micro/ctrl/cmd_vel topic set in SpotMicroCtrl node
    def cmd_vel_sub(self, msg):
        self.target_velocity = msg.data

    def run_10ms(self):
        pass

    def get_xy(self):
        return self.robot_leg_state
