from queue import Queue
from communication.zmq_client import Arx5Client, CTRL_DT
from peripherals.spacemouse_shared_memory import Spacemouse
from multiprocessing.managers import SharedMemoryManager
import numpy as np
import time


def start_teleop_recording(arx5_client: Arx5Client):

    ori_speed = 0.3
    pos_speed = 0.1
    gripper_speed = 0.04
    target_pose_6d = np.zeros((6,), dtype=np.float64)
    target_gripper_pos = 0.0

    window_size = 1
    spacemouse_queue = Queue(window_size)
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
                    arx5_client.reset_to_home()
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

                target_pose_6d[:3] += state[:3] * pos_speed * CTRL_DT
                target_pose_6d[3:] += state[3:] * ori_speed * CTRL_DT
                target_gripper_pos += gripper_cmd * gripper_speed * CTRL_DT
                if target_gripper_pos >= 1:
                    target_gripper_pos = 1
                elif target_gripper_pos <= 0:
                    target_gripper_pos = 0
                loop_cnt += 1
                while time.monotonic() < start_time + loop_cnt * CTRL_DT:
                    pass
                # print(
                #     f"Target pose: {target_pose_6d}, gripper pos: {target_gripper_pos:.3f}, gripper torque: {arx5_client.gripper_torque:.3f}"
                # )
                print(f"tcp pose: {arx5_client.tcp_pose}")
                arx5_client.set_ee_pose(target_pose_6d, target_gripper_pos)


if __name__ == "__main__":
    arx5_client = Arx5Client(zmq_ip="localhost", zmq_port=8765)
    arx5_client.reset_to_home()
    np.set_printoptions(precision=4, suppress=True)
    try:
        start_teleop_recording(arx5_client)
    except KeyboardInterrupt:
        print(f"Teleop recording is terminated. Resetting to home.")
        arx5_client.reset_to_home()
        arx5_client.set_to_damping()
