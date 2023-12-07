# install blinka
# https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
from board import SCL, SDA
import busio

# sudo pip3 install adafruit-circuitpython-pca9685
import adafruit_pca9685

from time import sleep


# thin wrapper for adafruit_pca9685.PCA9685
class pca9685:
    def __init__(self) -> None:
        try:
            self.i2c = busio.I2C(SCL, SDA)
        except Exception as e:
            raise Exception("Unable to initialize I2C. " + str(e))
        sleep(0.01)

        try:
            self.pca = adafruit_pca9685.PCA9685(self.i2c)
        except Exception as e:
            raise Exception("Unable to initialize PCA9685. " + str(e))
        sleep(0.01)

        self.pca.frequency = 50  # hz
        sleep(0.1)
        if abs(self.pca.frequency - 50) > 5:
            raise Exception("Unable to update PCA9685 frequency.")

    def __getitem__(self, index):
        return self.pca.channels[index].duty_cycle
    
    def __setitem__(self, index, value):
        try:
            self.pca.channels[index].duty_cycle = value
            return True
        except Exception as e:
            raise Exception("Unable to set duty cycle for PCA9685. " + str(e))
        