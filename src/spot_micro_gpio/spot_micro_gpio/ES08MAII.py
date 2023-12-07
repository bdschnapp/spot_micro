# For use with the PCA9685 servo header 
import numpy as np
from spot_micro_gpio.PCA9685 import pca9685

# TODO: Move constants into rosparam
DC_MIN = 2750
DC_MAX = 9000
RAD_MIN = 0
RAD_MAX = np.pi


class es08maii:
    def __init__(self,
                 pca: pca9685,
                 index: int,
                 dc_min: int,
                 dc_max: int,
                 rad_min: np.radians,
                 rad_max: np.radians,
                 rad_home: np.radians) -> None:
        self.pca = pca
        self.i = index

        self.dc_min = dc_min
        self.dc_max = dc_max

        self.theta = np.radians(0.0)
        self.home = np.radians(rad_home)
        self.rad_min = np.radians(rad_min)
        self.rad_max = np.radians(rad_max)

        self.set_theta(self.home)

    def _linear_scale(self, x: np.radians) -> int:
        # x = theta
        # y = dc
        # y = y0 + (x - x0)(y1 - y0)/(x1 - x0)
        # y0 = dc_min
        # y1 = dc_max
        # x0 = rad_min
        # x1 = rad_max
        return int(self.dc_min + ((x - self.rad_min) * (self.dc_max - self.dc_min) / (self.rad_max - self.rad_min)))

    def _set_dc(self, dc: int) -> bool:
        try:
            self.pca[self.i] = dc
            return True
        except Exception as e:
            raise Exception("Unable to set duty cycle. ", e)

    def set_theta(self, theta: np.radians) -> bool:
        if self.rad_max >= theta >= self.rad_min:
            if self._set_dc(self._linear_scale(theta)):
                self.theta = np.radians(theta)
                return True
        else:
            raise Exception(IndexError, "Theta out of bounds")
