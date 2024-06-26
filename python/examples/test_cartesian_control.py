import time
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import numpy as np

import arx5_interface as arx5


def main():
    controller = arx5.Arx5CartesianController(
        "L5", "can2", "../models/arx5_realsense.urdf"
    )
    controller.set_log_level(arx5.LogLevel.DEBUG)
    config = controller.get_robot_config()

    controller.reset_to_home()


if __name__ == "__main__":
    main()
