import os
import sys
import time
import click

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
def calibrate_joint(model: str, interface: str, joint_id: int):
    joint_controller = arx5.Arx5JointController(model, interface)
    gain = arx5.Gain(joint_controller.get_robot_config().joint_dof)
    joint_controller.set_gain(gain)
    joint_controller.enable_background_send_recv()
    joint_controller.calibrate_joint(joint_id)
    while True:
        state = joint_controller.get_state()
        pos = state.pos()
        print(", ".join([f"{x:.3f}" for x in pos]))
        time.sleep(0.1)

@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
def calibrate_gripper(model: str, interface: str):
    joint_controller = arx5.Arx5JointController(model, interface)
    joint_controller.calibrate_gripper()


if __name__ == "__main__":

    # calibrate_joint(0) # 0~5
    calibrate_gripper()
