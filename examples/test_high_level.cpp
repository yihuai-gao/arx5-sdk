#include <chrono>
#include <csignal>
#include "app/common.h"
#include "app/high_level.h"

using namespace arx;

Arx5HighLevel* arx5_high_level = new Arx5HighLevel("can0", "models/arx5.urdf");

void signal_handler(int signal) {
  std::cout << "SIGINT received" << std::endl;
  delete arx5_high_level;
  exit(signal);
}

int main() {
  HighState cmd;
  Arx5Solver solver("models/arx5.urdf");
  int loop_cnt = 0;
  Gain gain = Gain();
  // gain.kp = DEFAULT_KP / 30;
  gain.kd = DEFAULT_KD / 10;
  std::signal(SIGINT, signal_handler);
  arx5_high_level->set_gain(gain);
  arx5_high_level->set_high_cmd(cmd);
  while (true) {
    HighState high_state = arx5_high_level->get_high_state();
    JointState low_state = arx5_high_level->get_joint_state();
    loop_cnt++;
    // printf("raw x: %.2f, y: %.2f, z: %.2f, r: %.2f, p: %.2f, y: %.2f, gripper: %.2f\n", high_state.pose_6d[0], high_state.pose_6d[1], high_state.pose_6d[2], high_state.pose_6d[3], high_state.pose_6d[4], high_state.pose_6d[5], high_state.gripper_pos);
    printf("raw joint pos: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           low_state.pos[0], low_state.pos[1], low_state.pos[2],
           low_state.pos[3], low_state.pos[4], low_state.pos[5]);
    std::tuple<bool, Vec6d> result =
        solver.inverse_kinematics(high_state.pose_6d, low_state.pos);
    if (std::get<0>(result)) {
      Vec6d ik_joint_pos = std::get<1>(result);
      printf("ik  joint pos: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
             ik_joint_pos[0], ik_joint_pos[1], ik_joint_pos[2], ik_joint_pos[3],
             ik_joint_pos[4], ik_joint_pos[5]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
