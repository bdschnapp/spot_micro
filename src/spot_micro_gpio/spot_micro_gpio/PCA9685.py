# install blinka
# https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
from board import SCL, SDA
import busio
from queue import Queue
from threading import Thread
import time

# sudo pip3 install adafruit-circuitpython-pca9685
import adafruit_pca9685

from time import sleep

I2C_RATE = 0.0001 # s
PCA_FREQUENCY = 50 # hz

# Wrapper for adafruit_pca9685.PCA9685
class pca9685:
    def __init__(self, i2c_mux=None) -> None:
        self._shutdown = False
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

        self.pca.frequency = PCA_FREQUENCY  # hz
        sleep(0.1)
        if abs(self.pca.frequency - PCA_FREQUENCY) > 5:
            raise Exception("Unable to update PCA9685 frequency.")
        
        self.i2c_mux = i2c_mux
        self.i2c_queue = Queue()
        self.i2c_thread = Thread(target=self._i2c_write_func)
        self.i2c_thread.start()

    def _i2c_write_func(self):
        while not self._shutdown:
            with self.i2c_mux:
                index, value = self.i2c_queue.get()
                self.pca.channels[index].duty_cycle = value
                time.sleep(I2C_RATE)

    def shutdown(self):
        if not self._shutdown:
            self._shutdown = True
        if self.i2c_thread.is_alive():    
            self.i2c_thread.join()

    def __del__(self):
        self.shutdown()

    def __getitem__(self, index):
        return self.pca.channels[index].duty_cycle
    
    def __setitem__(self, index, value):
        if not self.i2c_thread.is_alive():
            raise Exception("I2C Write thread not alive")
        
        try:
            self.i2c_queue.put((index, value))
            return True
        except Exception as e:
            raise Exception("Unable to set duty cycle for PCA9685. " + str(e))
        