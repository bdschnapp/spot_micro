from spot_micro_gpio.PCA9685 import pca9685
from spot_micro_gpio.ES08MAII import es08maii
import numpy as np
import time


# TODO: Move constants into rosparam
DC_MIN = 2750
DC_MAX = 9000
RAD_MIN = 0
RAD_MAX = np.pi


def main():
    pca = pca9685()


    servos = []
    for i in range(8):
        servos.append(
            es08maii(
                pca,
                i * 2,
                DC_MIN,
                DC_MAX,
                RAD_MIN,
                RAD_MAX,
                RAD_MAX/2
            )
        )
        time.sleep(0.2)