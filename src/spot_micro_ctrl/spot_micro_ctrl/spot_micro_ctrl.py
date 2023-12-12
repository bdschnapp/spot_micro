import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, String

from spot_micro_ctrl.trajectory_generator import TrajectoryGenerator
from spot_micro_ctrl.stability_ctrl import StabilityController


class SpotMicroCtrlStateMachine:
    def __init__(self):
        self.trajectory_generator = TrajectoryGenerator()
        self.stability_controller = StabilityController()
        self.states = ['still', 'walk', 'trot', 'gallop']
        self.state = self.states[0]
        self.servo_msg = String()

    def state_update_sub(self, msg):
        if msg.data < 0 or msg.data > len(self.states):
            return False
        self.state = self.states[msg.data]
        self.trajectory_generator.update_state(self.state)
        self.stability_controller.update_state(self.state)

    def run_10ms(self):
        self.trajectory_generator.run_10ms()
        self.stability_controller.run_10ms()

        xy_target = self.stability_controller.get_xy() + self.trajectory_generator.get_xy()
        self.servo_msg = String()
        self.servo_msg.data = xy_target.calculate_angles()


class SpotMicroCtrl(Node):
    def __init__(self):
        super().__init__('SpotMicroCtrl')

        self.state_machine = SpotMicroCtrlStateMachine()

        self.state_sub = self.create_subscription(
            Int16,
            '/spot_micro/ctrl/state',
            self.state_machine.state_update_sub,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/spot_micro/ctrl/cmd_vel',
            self.state_machine.trajectory_generator.cmd_vel_sub,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/spot_micro/gpio/imu',
            self.state_machine.stability_controller.imu_sub,
            10
        )

        self.servo_ctrl_pub = self.create_publisher(
            String,
            '/spot_micro/gpio/servos',
            10
        )

        self.rate = 0.01  # 10 ms
        self.run10ms_timer = self.create_timer(self.rate, self._run_10ms)

    def _run_10ms(self):
        self.state_machine.run_10ms()
        self.servo_ctrl_pub.publish(self.state_machine.servo_msg)


def main(args=None):
    rclpy.init(args=args)
    ctrl_node = SpotMicroCtrl()
    rclpy.spin(ctrl_node)

    ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
