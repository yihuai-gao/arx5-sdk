import os
import sys
import time
ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5

def calibrate_joint(joint_id: int):
    joint_controller = arx5.Arx5JointController("can0")
    gain = arx5.Gain()
    joint_controller.set_gain(gain)
    joint_controller.enable_background_send_recv()
    joint_controller.calibrate_joint(joint_id)
    while True:
        state = joint_controller.get_state()
        pos = state.pos()
        print(", ".join([f"{x:.3f}" for x in pos]))
        time.sleep(0.1)

def calibrate_gripper():
    joint_controller = arx5.Arx5JointController("can0")
    joint_controller.calibrate_gripper()

if __name__ == "__main__":

    calibrate_joint(1)
    # calibrate_gripper()


