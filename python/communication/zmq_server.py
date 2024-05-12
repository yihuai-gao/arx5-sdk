import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import time
from typing import Any, cast
import arx5_interface as arx5
import zmq
from enum import IntEnum, auto
import sys
import traceback
import numpy as np


def echo_exception():
    exc_type, exc_value, exc_traceback = sys.exc_info()
    # Extract unformatted traceback
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    # Print line of code where the exception occurred

    return "".join(tb_lines)


class Arx5Server:
    def __init__(
        self,
        zmq_ip: str,
        zmq_port: int,
        no_cmd_timeout: float = 60.0,
    ):
        self.arx5_high_level = arx5.Arx5HighLevel()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://{zmq_ip}:{zmq_port}")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.zmq_ip = zmq_ip
        self.zmq_port = zmq_port
        self.last_cmd_time = time.monotonic()
        self.no_cmd_timeout = no_cmd_timeout
        self.is_reset_to_home = False

    def run(self):
        print(f"Arx5ZmqServer is running on {self.zmq_ip}:{self.zmq_port}")
        while True:
            try:
                socks = dict(self.poller.poll(int(self.no_cmd_timeout * 1000)))
                if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                    msg: dict[str, Any] = self.socket.recv_pyobj()
                    if self.arx5_high_level is None:
                        print(f"Reestablishing high level controller")
                        self.arx5_high_level = arx5.Arx5HighLevel()
                        self.arx5_high_level.set_log_level(self.log_level)
                else:

                    if self.arx5_high_level is not None:
                        print(
                            f"Timeout: No command received for {self.no_cmd_timeout} sec. ARX5 arm is reset to home position."
                        )
                        self.arx5_high_level.reset_to_home()
                        self.arx5_high_level.set_to_damping()
                        del self.arx5_high_level
                        self.arx5_high_level = None
                    continue
            except KeyboardInterrupt:
                break
            except Exception as e:
                exception_str = echo_exception()
                print(f"Error: {exception_str}")
                continue
            try:
                if not isinstance(msg, dict):
                    print(f"Error: Received invalid Message {msg}, ignored")
                    self.socket.send_pyobj(
                        {
                            "cmd": "UNKNOWN",
                            "data": f"ERROR: Received invalid Message {msg}",
                        }
                    )
                    continue
                if msg["cmd"] == "GET_STATE":
                    # print(f"Received GET_STATE message")
                    high_state = self.arx5_high_level.get_high_state()
                    low_state = self.arx5_high_level.get_low_state()
                    reply_msg = {
                        "cmd": "GET_STATE",
                        "data": {
                            "timestamp": high_state.timestamp,
                            "ee_pose": high_state.pose_6d().copy(),
                            "joint_pos": low_state.pos().copy(),
                            "joint_vel": low_state.vel().copy(),
                            "joint_torque": low_state.torque().copy(),
                            "gripper_pos": low_state.gripper_pos,
                            "gripper_vel": low_state.gripper_vel,
                            "gripper_torque": low_state.gripper_torque,
                        },
                    }
                    self.socket.send_pyobj(reply_msg)
                elif msg["cmd"] == "SET_EE_POSE":
                    # print(f"Received SET_EE_POSE message, data: {msg['data']}")
                    target_ee_pose = cast(np.ndarray, msg["data"]["ee_pose"])
                    if msg["data"]["gripper_pos"] is not None:
                        target_gripper_pos = cast(float, msg["data"]["gripper_pos"])
                    else:
                        # Maintain the current gripper position
                        target_gripper_pos = (
                            self.arx5_high_level.get_high_state().gripper_pos
                        )
                    if self.is_reset_to_home:
                        if np.linalg.norm(target_ee_pose) > 0.1:
                            error_str = f"Error: Cannot set EE pose far away from home: {target_ee_pose} after RESET_TO_HOME. Please check the input."
                            print(
                                error_str
                            )
                            self.socket.send_pyobj(
                                {
                                    "cmd": "SET_EE_POSE",
                                    "data": error_str,
                                }
                            )
                            continue

                    self.arx5_high_level.set_high_cmd(
                        arx5.HighState(target_ee_pose, target_gripper_pos)
                    )
                    high_state = self.arx5_high_level.get_high_state()
                    low_state = self.arx5_high_level.get_low_state()
                    reply_msg = {
                        "cmd": "SET_EE_POSE",
                        "data": {
                            "timestamp": high_state.timestamp,
                            "ee_pose": high_state.pose_6d().copy(),
                            "joint_pos": low_state.pos().copy(),
                            "joint_vel": low_state.vel().copy(),
                            "joint_torque": low_state.torque().copy(),
                            "gripper_pos": low_state.gripper_pos,
                            "gripper_vel": low_state.gripper_vel,
                            "gripper_torque": low_state.gripper_torque,
                        },
                    }
                    self.socket.send_pyobj(reply_msg)
                    self.is_reset_to_home = False
                elif msg["cmd"] == "RESET_TO_HOME":
                    print(f"Received RESET_TO_HOME message")
                    self.arx5_high_level.reset_to_home()
                    reply_msg = {
                        "cmd": "RESET_TO_HOME",
                        "data": "OK",
                    }
                    self.socket.send_pyobj(reply_msg)
                    self.is_reset_to_home = True
                elif msg["cmd"] == "SET_TO_DAMPING":
                    print(f"Received SET_TO_DAMPING message")
                    self.arx5_high_level.set_to_damping()
                    reply_msg = {
                        "cmd": "SET_TO_DAMPING",
                        "data": "OK",
                    }
                    self.socket.send_pyobj(reply_msg)
                    self.is_reset_to_home = False
                elif msg["cmd"] == "GET_GAIN":
                    print(f"Received GET_GAIN message")
                    gain = self.arx5_high_level.get_gain()
                    reply_msg = {
                        "cmd": "GET_GAIN",
                        "data": {
                            "kp": gain.kp().copy(),
                            "kd": gain.kd().copy(),
                            "gripper_kp": gain.gripper_kp,
                            "gripper_kd": gain.gripper_kd,
                        },
                    }
                    self.socket.send_pyobj(reply_msg)
                elif msg["cmd"] == "SET_GAIN":
                    print(f"Received SET_GAIN message, data: {msg['data']}")
                    assert isinstance(msg["data"], dict)

                    kp = cast(np.ndarray, msg["data"]["kp"])
                    kd = cast(np.ndarray, msg["data"]["kd"])
                    gripper_kp = cast(float, msg["data"]["gripper_kp"])
                    gripper_kd = cast(float, msg["data"]["gripper_kd"])
                    self.arx5_high_level.set_gain(
                        arx5.Gain(kp, kd, gripper_kp, gripper_kd)
                    )
                    reply_msg = {
                        "cmd": "SET_GAIN",
                        "data": "OK",
                    }
                    self.socket.send_pyobj(reply_msg)
                else:
                    raise ValueError(f"Unknown message type: {msg['cmd']}")
            except KeyboardInterrupt:
                break
            except Exception as e:
                exception_str = echo_exception()
                self.socket.send_pyobj(
                    {"cmd": msg["cmd"], "data": f"ERROR: {exception_str}"}
                )
                print(f"Error: {exception_str}")
                continue

    def __del__(self):
        self.socket.close()
        self.context.term()
        print("Arx5ZmqServer is terminated")


if __name__ == "__main__":
    server = Arx5Server(zmq_ip="0.0.0.0", zmq_port=8765)
    server.run()
