from queue import Queue
import os
import sys

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
from arx5_interface import Arx5CartesianController, EEFState, Gain, LogLevel
from peripherals.spacemouse_shared_memory import Spacemouse
from multiprocessing.managers import SharedMemoryManager

import time
import click


def start_teleop_recording(controller: Arx5CartesianController):

    ori_speed = 0.8
    pos_speed = 0.3
    gripper_speed = 0.04
    target_pose_6d = np.zeros((6,), dtype=np.float64)
    target_gripper_pos = 0.0

    window_size = 20
    spacemouse_queue = Queue(window_size)
    robot_config = controller.get_robot_config()
    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager, deadzone=0.3, max_value=500) as sm:

            def get_filtered_spacemouse_output(sm: Spacemouse):
                state = sm.get_motion_state_transformed()
                if (
                    spacemouse_queue.maxsize > 0
                    and spacemouse_queue._qsize() == spacemouse_queue.maxsize
                ):
                    spacemouse_queue._get()
                spacemouse_queue.put_nowait(state)
                return np.mean(np.array(list(spacemouse_queue.queue)), axis=0)

            print("Teleop tracking ready. Waiting for spacemouse movement to start.")

            while True:
                button_left = sm.is_button_pressed(0)
                button_right = sm.is_button_pressed(1)
                state = get_filtered_spacemouse_output(sm)
                if state.any() or button_left or button_right:
                    print(f"Start tracking!")
                    break
            directions = np.zeros(6, dtype=np.float64)
            start_time = time.monotonic()
            loop_cnt = 0
            while True:

                print(
                    f"Time elapsed: {time.monotonic() - start_time:.03f}s",
                    end="\r",
                )
                # Spacemouse state is in the format of (x y z roll pitch yaw)
                prev_directions = directions
                directions = np.zeros(7, dtype=np.float64)
                state = get_filtered_spacemouse_output(sm)
                button_left = sm.is_button_pressed(0)
                button_right = sm.is_button_pressed(1)
                if button_left and button_right:
                    controller.reset_to_home()
                    gain = Gain()
                    gain.kp()[:] = np.array([150.0, 150.0, 200.0, 60.0, 30.0, 30.0])
                    gain.kd()[:] = np.array([5.0, 5.0, 5.0, 1.5, 1.5, 1.5])
                    gain.gripper_kp = robot_config.default_gripper_kp
                    gain.gripper_kd = robot_config.default_gripper_kd

                    controller.set_gain(gain)
                    target_pose_6d = np.zeros((6,), dtype=np.float64)
                    target_gripper_pos = 0.0
                    loop_cnt = 0
                    start_time = time.monotonic()
                    continue
                elif button_left and not button_right:
                    gripper_cmd = 1
                elif button_right and not button_left:
                    gripper_cmd = -1
                else:
                    gripper_cmd = 0

                target_pose_6d[:3] += state[:3] * pos_speed * robot_config.controller_dt
                target_pose_6d[3:] += state[3:] * ori_speed * robot_config.controller_dt
                target_gripper_pos += (
                    gripper_cmd * gripper_speed * robot_config.controller_dt
                )
                if target_gripper_pos >= robot_config.gripper_width:
                    target_gripper_pos = robot_config.gripper_width
                elif target_gripper_pos <= 0:
                    target_gripper_pos = 0
                loop_cnt += 1
                while (
                    time.monotonic()
                    < start_time + loop_cnt * robot_config.controller_dt
                ):
                    pass

                eef_cmd = EEFState()
                eef_cmd.pose_6d()[:] = target_pose_6d
                eef_cmd.gripper_pos = target_gripper_pos
                controller.set_eef_cmd(eef_cmd)


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("can_interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5.urdf", help="URDF file path")
def main(model: str, can_interface: str, urdf_path: str):
    controller = Arx5CartesianController(model, can_interface, urdf_path)
    controller.reset_to_home()

    gain = Gain()
    gain.kp()[:] = np.array([150.0, 150.0, 200.0, 60.0, 30.0, 30.0])
    gain.kd()[:] = np.array([5.0, 5.0, 5.0, 1.5, 1.5, 1.5])
    robot_config = controller.get_robot_config()
    gain.gripper_kp = robot_config.default_gripper_kp
    gain.gripper_kd = robot_config.default_gripper_kd
    controller.set_gain(gain)
    controller.set_log_level(LogLevel.DEBUG)
    np.set_printoptions(precision=4, suppress=True)
    try:
        start_teleop_recording(controller)
    except KeyboardInterrupt:
        print(f"Teleop recording is terminated. Resetting to home.")
        controller.reset_to_home()
        controller.set_to_damping()


if __name__ == "__main__":
    main()
