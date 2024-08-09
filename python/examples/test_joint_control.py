import time

import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click
import numpy as np


def easeInOutQuad(t):
    t *= 2
    if t < 1:
        return t * t / 2
    else:
        t -= 1
        return -(t * (t - 2) - 1) / 2


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("can_interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5.urdf", help="URDF file path")
def main(model: str, can_interface: str, urdf_path: str):
    np.set_printoptions(precision=3, suppress=True)
    arx5_joint_controller = arx5.Arx5JointController(model, can_interface)
    arx5_joint_controller.set_log_level(arx5.LogLevel.DEBUG)
    config = arx5_joint_controller.get_robot_config()

    step_num = 1500
    USE_MULTITHREADING = False
    if USE_MULTITHREADING:
        arx5_joint_controller.enable_background_send_recv()
    arx5_joint_controller.reset_to_home()
    arx5_joint_controller.enable_gravity_compensation(urdf_path)

    target_joint_poses = np.array([1.0, 2.0, 2.0, 1.5, 1.5, -1.57])
    gain = arx5.Gain(config.joint_dof)
    gain.gripper_kp = 5.0
    gain.gripper_kd = config.default_gripper_kd

    gain.kp()[:] = np.array([100.0, 100.0, 100.0, 30.0, 30, 5.0])
    gain.kd()[:] = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    arx5_joint_controller.set_gain(gain)

    for i in range(step_num):
        cmd = arx5.JointState(config.joint_dof)
        # i = 0
        cmd.pos()[0:4] = easeInOutQuad(float(i) / step_num) * target_joint_poses[0:4]
        cmd.gripper_pos = easeInOutQuad((i / (step_num - 1))) * 0.08
        arx5_joint_controller.set_joint_cmd(cmd)
        if not USE_MULTITHREADING:
            arx5_joint_controller.send_recv_once()
        else:
            time.sleep(config.controller_dt)
        JointState = arx5_joint_controller.get_state()
        arm_dof_pos = JointState.pos().copy()
        arm_dof_vel = JointState.vel().copy()
        # print(arm_dof_pos, arm_dof_vel)
        # print(f"gripper: {JointState.gripper_pos:.05f}")

    for i in range(step_num):
        cmd = arx5.JointState(config.joint_dof)
        cmd.pos()[0:4] = (
            easeInOutQuad((1 - float(i) / step_num)) * target_joint_poses[0:4]
        )
        cmd.gripper_pos = easeInOutQuad((1 - i / (step_num - 1))) * 0.08
        arx5_joint_controller.set_joint_cmd(cmd)
        if not USE_MULTITHREADING:
            arx5_joint_controller.send_recv_once()
        else:
            time.sleep(config.controller_dt)
        JointState = arx5_joint_controller.get_state()
        # print(f"gripper: {JointState.gripper_pos:.05f}")


main()
