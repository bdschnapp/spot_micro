import rclpy
from sensor_msgs.msg import Imu

# sudo pip3 install adafruit-circuitpython-bno055
import adafruit_bno055


class bno055:
    def __init__(self, i2c):
        self.imu = adafruit_bno055.BNO055_I2C(i2c)
        self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        self.imu_data_seq_counter = 0

    def run_10ms(self):
        pass

    def get_imu(self):
        imu_msg = Imu()

        quaternion = self.imu.quaternion
        linear_acceleration = self.imu.linear_acceleration
        gyroscope = self.imu.gyro

        imu_msg.header.stamp = rclpy.Time.now()
        imu_msg.header.frame_id = '/imu'
        imu_msg.header.seq = self.imu_data_seq_counter

        imu_msg.orientation.w = quaternion[0]
        imu_msg.orientation.x = quaternion[1]
        imu_msg.orientation.y = quaternion[2]
        imu_msg.orientation.z = quaternion[3]

        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]

        imu_msg.angular_velocity.x = gyroscope[0]
        imu_msg.angular_velocity.y = gyroscope[1]
        imu_msg.angular_velocity.z = gyroscope[2]

        imu_msg.orientation_covariance[0] = 0.03
        imu_msg.orientation_covariance[4] = 0.03
        imu_msg.orientation_covariance[8] = 0.03
        imu_msg.linear_acceleration_covariance[0] = 1.0
        imu_msg.linear_acceleration_covariance[4] = 1.0
        imu_msg.linear_acceleration_covariance[8] = 1.0
        imu_msg.angular_velocity_covariance[0] = 0.1
        imu_msg.angular_velocity_covariance[4] = 0.1
        imu_msg.angular_velocity_covariance[8] = 0.1

        self.imu_data_seq_counter += 1
