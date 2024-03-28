import os
os.environ['ROS_ENV'] = 'True'

import rclpy
from rclpy.node import Node

from spot_micro_gpio.PCA9685 import pca9685
from spot_micro_gpio.ES08MAII import es08maii
from spot_micro_gpio.BNO055 import bno055

from sensor_msgs.msg import Imu
from std_msgs.msg import String

from threading import Lock


class SpotMicroGPIO(Node):
    def __init__(self):
        super().__init__('SpotMicroGPIO')
        
        self.i2c_mux = Lock()
        self.pca = pca9685(self.i2c_mux)

        self.declare_parameter('/spot_micro/cfg/dc_min', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('/spot_micro/cfg/dc_max', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('/spot_micro/cfg/rad_min', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('/spot_micro/cfg/rad_max', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('/spot_micro/cfg/rad_home', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.dc_min = self.get_parameter('/spot_micro/cfg/dc_min').value
        self.dc_max = self.get_parameter('/spot_micro/cfg/dc_max').value
        self.rad_min = self.get_parameter('/spot_micro/cfg/rad_min').value
        self.rad_max = self.get_parameter('/spot_micro/cfg/rad_max').value
        self.rad_home = self.get_parameter('/spot_micro/cfg/rad_home').value

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

        # self.imu = bno055(self.pca.i2c)

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

        self.rate = 0.01  # 10 ms
        self.run10ms_timer = self.create_timer(self.rate, self._run_10ms)

    def _servo_sub(self, msg):
        self.get_logger().info('/spot_micro/gpio/servos: ' + str(msg.data))
        self.servo_target = msg.data

    def _imu_pub(self):
        self.imu_publisher.publish(self.imu.get_imu())

    def _run_10ms(self):
        # publish updated Imu data
        # self.imu.run_10ms()
        # self._imu_pub()

        # control servos
        if self.servo_target:
            for index, value in enumerate(self.servo_target.split(',')):
                try:
                    if abs(self.servos[index].theta - float(value)) > 0.0005:    
                        self.servos[index].set_theta(float(value))
                except Exception as e:
                    self.get_logger().error(str(e) + '\n' +
                                            'set theta failed, index:' + str(index) + 
                                            " value:" + str(float(value)) + 
                                            " rad_max:" + str(self.servos[index].rad_max) + 
                                            " rad_min:" + str(self.servos[index].rad_min)
                                            )


def main(args=None):
    rclpy.init(args=args)
    gpio_node = SpotMicroGPIO()
    rclpy.spin(gpio_node)

    gpio_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
