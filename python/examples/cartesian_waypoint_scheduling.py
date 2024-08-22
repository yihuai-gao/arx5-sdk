import time

import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click
import numpy as np


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5.urdf", help="URDF file path")
def main(model: str, interface: str, urdf_path: str):
    controller = arx5.Arx5CartesianController(model, interface, urdf_path)

    controller.set_log_level(arx5.LogLevel.DEBUG)
    home_pose = controller.get_home_pose()
    controller.reset_to_home()
    cartesian_waypoints = home_pose + np.array(
        [
            [0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
            [0.3, 0.0, 0.3, 0.0, 0.0, 0.0],
            [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
    )
    waypoint_interval_s = 2.0
    for waypoint in cartesian_waypoints:
        eef_state = controller.get_eef_state()
        eef_cmd = arx5.EEFState(waypoint, 0.0)
        eef_cmd.timestamp = eef_state.timestamp + waypoint_interval_s
        controller.set_eef_cmd(eef_cmd)
        time.sleep(waypoint_interval_s)

    controller.reset_to_home()


if __name__ == "__main__":
    main()
