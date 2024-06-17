#include "app/joint_controller.h"
#include <unistd.h>
#include <chrono>
#include <csignal>

using namespace arx;

Arx5JointController *arx5_joint_controller = new Arx5JointController("can0");

void signal_handler(int signal)
{
    std::cout << "SIGINT received" << std::endl;
    delete arx5_joint_controller;
    exit(signal);
}

int main()
{
    std::signal(SIGINT, signal_handler);
    arx5_joint_controller->enable_background_send_recv();
    JointState cmd;
    int loop_cnt = 0;
    while (true)
    {
        JointState state = arx5_joint_controller->get_state();
        std::cout << "Gripper: " << state.gripper_pos << " Pos: " << state.pos[0] << " " << state.pos[1] << " " << state.pos[2] << " " << state.pos[3] << " " << state.pos[4] << " " << state.pos[5] << " " << std::endl;
        usleep(10000); // 10ms
        loop_cnt++;
        if (loop_cnt % 200 == 0)
        {
            arx5_joint_controller->reset_to_home();
            arx5_joint_controller->set_to_damping();
        }
    }
    return 0;
}