import time
import numpy as np
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
print(ROOT_DIR)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5

np.set_printoptions(precision=3, suppress=True)
arx5_low_level = arx5.Arx5LowLevel()

arx5_low_level.enable_background_send_recv()
arx5_low_level.set_log_level(arx5.LogLevel.DEBUG)
gain = arx5.Gain()
gain.kd()[:] = arx5.DEFAULT_KD
arx5_low_level.set_gain(gain)
arx5_low_level.reset_to_home()
# while True:
#     state = arx5_low_level.get_state()
#     print(f"pos: {state.pos}")
#     time.sleep(0.1)

target_joint_poses = [2, 0.0, 1.3, 1.5, 1.5, 1.5]
# for joint_id in [0, 2, 3, 4, 5]:
joint_id = 3
gain = arx5.Gain()
gain.kd()[:] = arx5.DEFAULT_KD
gain.kp()[joint_id] = arx5.DEFAULT_KP[joint_id]
gain.kp()[3:] = arx5.DEFAULT_KP[3:]
arx5_low_level.set_gain(gain)
step_num = 1000  # 5s
for i in range(step_num):
    cmd = arx5.LowState()
    cmd.pos()[joint_id] = float(i) / step_num * target_joint_poses[joint_id]
    arx5_low_level.set_low_cmd(cmd)
    lowstate = arx5_low_level.get_state()
    print(f"torque: {lowstate.torque()}")
    time.sleep(arx5.CTRL_DT)

for i in range(step_num):
    cmd = arx5.LowState()
    cmd.pos()[joint_id] = (1 - float(i) / step_num) * target_joint_poses[joint_id]
    arx5_low_level.set_low_cmd(cmd)
    time.sleep(arx5.CTRL_DT)
