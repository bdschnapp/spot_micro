import os
os.environ['ROS_ENV'] = 'True'

import rclpy
from rclpy.node import Node

from spot_micro_gpio.PCA9685 import pca9685
from spot_micro_gpio.ES08MAII import es08maii
from spot_micro_gpio.BNO055 import bno055

from sensor_msgs.msg import Imu
from std_msgs.msg import String


class SpotMicroGPIO(Node):
    def __init__(self):
        super().__init__('SpotMicroGPIO')
        self.pca = pca9685()

        self.dc_min = rclpy.get_param('/spot_micro/cfg/dc_min')
        self.dc_max = rclpy.get_param('/spot_micro/cfg/dc_max')
        self.rad_min = rclpy.get_param('/spot_micro/cfg/rad_min')
        self.rad_max = rclpy.get_param('/spot_micro/cfg/rad_max')
        self.rad_home = rclpy.get_param('/spot_micro/cfg/rad_home')

        self.servos = []
        self.servo_target = None
        for i in range(8):
            self.servos.append(
                es08maii(
                    self.pca,
                    i * 2,
                    self.dc_min[i],
                    self.dc_max[i],
                    self.rad_min[i],
                    self.rad_max[i],
                    self.rad_home[i]
                )
            )

        self.imu = bno055(self.pca.i2c)

        self.imu_publisher = self.create_publisher(
            Imu,
            '/spot_micro/gpio/imu',
            10
        )

        self.servo_subscriber = self.create_subscription(
            String,
            '/spot_micro/gpio/servos',
            self._servo_sub,
            10
        )

    def _servo_sub(self, msg):
        self.servo_target = msg.data

    def _imu_pub(self):
        self.imu_publisher.publish(self.imu.get_imu())

    def run_10ms(self):
        # publish updated Imu data
        self._imu_pub()

        # control servos
        for value, index in enumerate(self.servo_target.split(',')):
            self.servos[index].set_theta(float(value))


def main(args=None):
    rclpy.init(args=args)
    gpio_node = SpotMicroGPIO()
    rclpy.spin(gpio_node)

    gpio_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
