import os
ROS_ENV = os.environ['ROS_ENV'] == 'True'

if ROS_ENV:
    import rclpy
    from sensor_msgs.msg import Imu
else:
    import json

import time

# sudo pip3 install adafruit-circuitpython-bno055
import adafruit_bno055

if ROS_ENV:
    class bno055:
        def __init__(self, i2c, i2c_mux=None):
            self.imu = adafruit_bno055.BNO055_I2C(i2c)
            self.imu.mode = adafruit_bno055.IMUPLUS_MODE
            self.imu_data_seq_counter = 0
            self.i2c_mux = i2c_mux
            self.quaternion = None
            self.linear_acceleration = None
            self.gyroscope = None

        def run_10ms(self):
            with self.i2c_mux:
                self.quaternion = self.imu.quaternion
                self.linear_acceleration = self.imu.linear_acceleration
                self.gyroscope = self.imu.gyro
                time.sleep(0.0001)

        def get_imu(self):
            imu_msg = Imu()

            imu_msg.header.stamp = rclpy.Time.now()
            imu_msg.header.frame_id = '/imu'
            imu_msg.header.seq = self.imu_data_seq_counter

            imu_msg.orientation.w = self.quaternion[0]
            imu_msg.orientation.x = self.quaternion[1]
            imu_msg.orientation.y = self.quaternion[2]
            imu_msg.orientation.z = self.quaternion[3]

            imu_msg.linear_acceleration.x = self.linear_acceleration[0]
            imu_msg.linear_acceleration.y = self.linear_acceleration[1]
            imu_msg.linear_acceleration.z = self.linear_acceleration[2]

            imu_msg.angular_velocity.x = self.gyroscope[0]
            imu_msg.angular_velocity.y = self.gyroscope[1]
            imu_msg.angular_velocity.z = self.gyroscope[2]

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

            return imu_msg

else:  # os.environ['ROS_ENV'] == 'False'
    class bno055:
        def __init__(self, i2c, i2c_mux=None):
            self.imu = adafruit_bno055.BNO055_I2C(i2c)
            self.imu.mode = adafruit_bno055.IMUPLUS_MODE
            self.imu_data_seq_counter = 0
            self.i2c_mux = i2c_mux
            self.quaternion = None
            self.linear_acceleration = None
            self.gyroscope = None

        def run_10ms(self):
            with self.i2c_mux:
                self.quaternion = self.imu.quaternion
                self.linear_acceleration = self.imu.linear_acceleration
                self.gyroscope = self.imu.gyro
                time.sleep(0.0001)

        def get_imu(self):
            imu_msg_dict = {}
            imu_msg_dict["header"] = {}
            imu_msg_dict["header"]["stamp"] = time.now()
            imu_msg_dict["header"]["frame_id"] = '/imu'
            imu_msg_dict["header"]["seq"] = self.imu_data_seq_counter

            imu_msg_dict['orientation'] = {}
            imu_msg_dict['orientation']['w'] = self.quaternion[0]
            imu_msg_dict['orientation']['x'] = self.quaternion[1]
            imu_msg_dict['orientation']['y'] = self.quaternion[2]
            imu_msg_dict['orientation']['z'] = self.quaternion[3]

            imu_msg_dict['linear_acceleration'] = {}
            imu_msg_dict['linear_acceleration']['x'] = self.linear_acceleration[0]
            imu_msg_dict['linear_acceleration']['y'] = self.linear_acceleration[1]
            imu_msg_dict['linear_acceleration']['z'] = self.linear_acceleration[2]

            imu_msg_dict['angular_velocity'] = {}
            imu_msg_dict['angular_velocity']['x'] = self.gyroscope[0]
            imu_msg_dict['angular_velocity']['y'] = self.gyroscope[1]
            imu_msg_dict['angular_velocity']['z'] = self.gyroscope[2]

            imu_msg_dict['orientation_covariance'] = [0 for _ in range(9)]
            imu_msg_dict['linear_acceleration_covariance'] = [0 for _ in range(9)]
            imu_msg_dict['angular_velocity_covariance'] = [0 for _ in range(9)]

            imu_msg_dict['orientation_covariance'][0] = 0.03
            imu_msg_dict['orientation_covariance'][4] = 0.03
            imu_msg_dict['orientation_covariance'][8] = 0.03

            imu_msg_dict['linear_acceleration_covariance'][0] = 1.0
            imu_msg_dict['linear_acceleration_covariance'][4] = 1.0
            imu_msg_dict['linear_acceleration_covariance'][8] = 1.0

            imu_msg_dict['angular_velocity_covariance'][0] = 0.1
            imu_msg_dict['angular_velocity_covariance'][4] = 0.1
            imu_msg_dict['angular_velocity_covariance'][8] = 0.1

            self.imu_data_seq_counter += 1

            imu_msg_json = json.dumps(imu_msg_dict)
            return imu_msg_json