import time
import numpy as np
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5


def easeInOutQuad(t):
    t *= 2
    if t < 1:
        return t * t / 2
    else:
        t -= 1
        return -(t * (t - 2) - 1) / 2


def main():
    np.set_printoptions(precision=3, suppress=True)
    arx5_0 = arx5.Arx5LowLevel("can0")
    arx5_1 = arx5.Arx5LowLevel("can1")

    arx5_0.enable_background_send_recv()
    arx5_0.reset_to_home()
    arx5_1.enable_background_send_recv()
    arx5_1.reset_to_home()

    target_joint_poses = np.array([1.0, 2.0, 2.0, 1.5, 1.5, -1.57])
    gain = arx5.Gain()
    gain.gripper_kp = 5.0
    gain.gripper_kd = arx5.DEFAULT_GRIPPER_KD

    # gain.kp()[:] = [30.0, 30.0, 30.0, 5.0, 5.0, 3.0]
    # gain.kd()[:] = [0.001, 0.01, 0.01, 0.01, 0.01, 0.01]

    gain.kp()[:] = [70.0, 70.0, 70.0, 30.0, 30, 5.0]
    gain.kd()[:] = [10.0, 10.0, 10.0, 1.0, 1.0, 0.5]

    arx5_0.set_gain(gain)
    step_num = 800  # 5s
    USE_TIMER = True
    if not USE_TIMER:
        arx5_0.disable_background_send_recv()

    for i in range(step_num):
        cmd = arx5.LowState()
        # i = 0
        cmd.pos()[0:4] = easeInOutQuad(float(i) / step_num) * target_joint_poses[0:4]
        cmd.gripper_pos = easeInOutQuad((i / (step_num - 1))) * 0.08
        arx5_0.set_low_cmd(cmd)
        arx5_1.set_low_cmd(cmd)
        if not USE_TIMER:
            arx5_0.send_recv_once()
            arx5_1.send_recv_once()
        lowstate = arx5_0.get_state()
        lowstate = arx5_1.get_state()
        arm_dof_pos = lowstate.pos().copy()
        arm_dof_vel = lowstate.vel().copy()
        # print(arm_dof_pos, arm_dof_vel)
        # print(f"gripper: {lowstate.gripper_pos:.05f}")
        time.sleep(arx5.LOW_LEVEL_DT)

    for i in range(step_num):
        cmd = arx5.LowState()
        cmd.pos()[0:4] = (
            easeInOutQuad((1 - float(i) / step_num)) * target_joint_poses[0:4]
        )
        cmd.gripper_pos = easeInOutQuad((1 - i / (step_num - 1))) * 0.08
        arx5_0.set_low_cmd(cmd)
        arx5_1.set_low_cmd(cmd)
        if not USE_TIMER:
            arx5_0.send_recv_once()
            arx5_1.send_recv_once()
        time.sleep(arx5.LOW_LEVEL_DT)
        lowstate = arx5_0.get_state()
        lowstate = arx5_1.get_state()
        # print(f"gripper: {lowstate.gripper_pos:.05f}")


main()