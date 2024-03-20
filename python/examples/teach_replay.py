from communication.zmq_client import Arx5Client, CTRL_DT
import time
from peripherals.keystroke_counter import KeystrokeCounter, KeyCode
import numpy as np
import os, sys


def start_teaching(client: Arx5Client, data_file: str):
    client.reset_to_home()
    client.set_to_damping()
    gain = client.get_gain()
    gain["kd"] = 0.1 * gain["kd"]
    client.set_gain(gain)  # set to passive

    print("Teaching mode ready. Press 't' to start teaching.")
    teaching_started = False
    traj = []
    start_time = 0
    with KeystrokeCounter() as key_counter:
        while True:
            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char="t"):
                    if teaching_started:
                        print(f"Teaching is already started!")
                        continue
                    print("Teaching started! Press 'q' to quit teaching.")
                    teaching_started = True
                    start_time = time.monotonic()
                elif key_stroke == KeyCode(char="q"):
                    print(f"Teaching stopped! Trajectory saved to file {data_file}")
                    teaching_started = False
                    np.save(data_file, traj, allow_pickle=True)
                    return
            if teaching_started:
                state = client.get_state()
                state["timestamp"] = time.monotonic() - start_time
                traj.append(state)
                time.sleep(CTRL_DT)
                # print(f"Time elapsed: {time.monotonic() - start_time:.03f}s", end="\r")
                print(f"tcp pose: {arx5_client.tcp_pose}")


def start_high_level_replay(client: Arx5Client, data_file: str):
    client.reset_to_home()
    traj = np.load(data_file, allow_pickle=True)
    replay_started = False
    start_time = 0
    loop_cnt = 0
    print("Replay mode ready. Press 'r' to start replay.")
    with KeystrokeCounter() as key_counter:
        while True:
            press_events = key_counter.get_press_events()
            for key_stroke in press_events:
                if key_stroke == KeyCode(char="r"):
                    if replay_started:
                        print(f"Replay is already started!")
                        continue
                    print("Replay started! Press 'q' to quit replay.")
                    replay_started = True
                    start_time = time.monotonic()
                    loop_cnt = 0
                elif key_stroke == KeyCode(char="q"):
                    print(f"Replay stopped!")
                    return
            if replay_started:
                if loop_cnt < len(traj):
                    target_state = traj[loop_cnt]
                    loop_cnt += 1
                    client.set_ee_pose(
                        target_state["ee_pose"], target_state["gripper_pos"]
                    )
                    print(
                        f"Time elapsed: {time.monotonic() - start_time:.03f}s/{traj[-1]['timestamp']:.03f}",
                        end="\r",
                    )
                else:
                    print(f"\nReplay finished!")
                    return
                time.sleep(CTRL_DT * 2)


if __name__ == "__main__":
    arx5_client = Arx5Client(zmq_ip="localhost", zmq_port=8765)
    np.set_printoptions(precision=4, suppress=True)
    os.makedirs("data", exist_ok=True)
    start_teaching(arx5_client, "data/teach_traj.npy")
    # start_high_level_replay(arx5_client, "data/teach_traj.npy")
