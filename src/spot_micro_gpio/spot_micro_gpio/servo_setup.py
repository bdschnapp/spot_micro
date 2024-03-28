import os
os.environ['ROS_ENV'] = 'False'

from PCA9685 import pca9685
from ES08MAII import es08maii
from threading import Lock
import numpy as np
import time


# TODO: Move constants into rosparam
DC_MIN = [2750, 2750, 2750, 2870, 2750, 2750, 2750, 2750]
DC_MAX = [9000, 9000, 9000, 9000, 9000, 9000, 9000, 9000]
RAD_MIN = 0
RAD_MAX = np.pi
RAD_HOME = [1.6707, 1.4707, 1.6707, 1.1707, 1.5707, 1.6707, 1.5707, 1.4707]


def main():
    i2c_mux = Lock()
    pca = pca9685(i2c_mux)
    time.sleep(1)

    servos = []
    for i in range(8):
        servos.append(
            es08maii(
                pca,
                i * 2,
                DC_MIN[i],
                DC_MAX[i],
                RAD_MIN,
                RAD_MAX,
                RAD_HOME[i]
            )
        )
        time.sleep(0.5)

    index = 0
    while index >= 0:
        index = input("Index: ")
        dc = input("DC: ")
        servos[str(index)]._set_dc(int(dc))


if __name__ == "__main__":
    main()
