import time
import numpy as np
import numpy.typing as npt
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5

np.set_printoptions(precision=6, suppress=True)
arx5_high_level = arx5.Arx5HighLevel()
arx5_high_level.set_log_level(arx5.LogLevel.DEBUG)
gain = arx5_high_level.get_gain()
gain.kd()[:] = gain.kd() * 0.01
arx5_high_level.set_gain(gain)
solver = arx5.Arx5Solver()
solver.init_solver("../models/arx5_gopro.urdf")
# while True:
#     high_state = arx5_high_level.get_high_state()
#     low_state = arx5_high_level.get_joint_state()
#     print(f"low_state: {low_state.pos()}")
#     # print(f"pose_6d: {state.pose_6d()}")
#     est_torque = solver.inverse_dynamics(
#         low_state.pos(), np.zeros(6, dtype=np.float64), np.zeros(6, dtype=np.float64)
#     )
#     # print(f"actual torque: {low_state.torque()} estimated torque: {est_torque}")
#     time.sleep(0.1)


def run_gripper():
    arx5_high_level.reset_to_home()
    step_num = 1000
    start_time = time.monotonic()
    for i in range(step_num):
        gripper_pos = float(i) / step_num
        high_cmd = arx5.HighState(np.zeros(6, dtype=np.float64), gripper_pos)
        arx5_high_level.set_high_cmd(high_cmd)
        print(f"Step {i + 1}/{step_num}")
        while time.monotonic() - start_time < (i + 1) * arx5.CTRL_DT:
            pass


def run_cartesian_control():
    home_pose = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
    target_pose = np.array([0.25, 0.25, 0.25, 0, 0, 0], dtype=np.float64)
    arx5_high_level.reset_to_home()

    def move_to_pose(
        start_pose: npt.NDArray[np.float64],
        stop_pose: npt.NDArray[np.float64],
        duration: float,
    ):
        step_num = int(duration / arx5.CTRL_DT)
        start_time = time.monotonic()
        for i in range(step_num):
            interp_pose = stop_pose * (i + 1) / step_num + start_pose * (
                1 - (i + 1) / step_num
            )
            high_cmd = arx5.HighState(interp_pose, 0)
            arx5_high_level.set_high_cmd(high_cmd)
            print(f"Step {i + 1}/{step_num}")
            while time.monotonic() - start_time < (i + 1) * arx5.CTRL_DT:
                pass

    move_to_pose(home_pose, target_pose, 5.0)
    move_to_pose(target_pose, home_pose, 5.0)

    target_pose = np.array([0.25, -0.25, 0.25, 0, 0, 0], dtype=np.float64)

    move_to_pose(home_pose, target_pose, 5.0)
    move_to_pose(target_pose, home_pose, 5.0)


if __name__ == "__main__":
    # run_gripper()
    run_cartesian_control()
