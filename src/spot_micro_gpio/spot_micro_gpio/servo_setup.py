import os
os.environ['ROS_ENV'] = 'False'

from PCA9685 import pca9685
from ES08MAII import es08maii
from threading import Lock
import numpy as np
import time


DC_MIN = [4300, 3500, 4400, 3400, 7500, 7500, 7550, 7500]
DC_MAX = [8750, 7000, 8750, 7000, 3500, 4000, 3500, 4000]
RAD_MIN = [0, 0.7854, 0, 0.7854, 0, 0.7854, 0, 0.7854]
RAD_MAX = [2.1025, 2.3562, 2.1354, 2.3562, 1.9639, 2.3562, 1.9692, 2.3562]
RAD_HOME = [0.5, 2, 0.75, 1.5, 0.75, 1.5, 0.5, 2]


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
                RAD_MIN[i],
                RAD_MAX[i],
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
