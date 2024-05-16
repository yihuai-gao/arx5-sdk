#include "app/joint_controller.h"
#include <unistd.h>

void test_gripper_movement(Arx5JointController &arx5_joint_controller)
{
    arx5_joint_controller.reset_to_home();
    JointState cmd;
    // Open the gripper in 1s
    int step_num = 400;
    double current_gripper_pos = arx5_joint_controller.get_state().gripper_pos;
    arx5_joint_controller.enable_background_send_recv();
    for (int i = 0; i <= step_num; ++i)
    {
        float alpha = double(i) / double(step_num);
        cmd.gripper_pos = current_gripper_pos * (1 - alpha) + GRIPPER_WIDTH * alpha;
        arx5_joint_controller.set_joint_cmd(cmd);
        usleep(5000);
    }
    usleep(500000); // Hardcode 0.5 s
    // Close the gripper in 1s
    for (int i = 0; i <= step_num; ++i)
    {
        float alpha = double(i) / double(step_num);
        cmd.gripper_pos = GRIPPER_WIDTH * (1 - alpha) + 0 * alpha;
        arx5_joint_controller.set_joint_cmd(cmd);
        usleep(5000);
    }
    usleep(500000); // Hardcode 0.5 s
    arx5_joint_controller.set_to_damping();
    arx5_joint_controller.disable_background_send_recv();
}

int main()
{
    Arx5JointController arx5_joint_controller;
    int loop_cnt = 0;
    arx5_joint_controller.calibrate_gripper();
    // test_gripper_movement(arx5_joint_controller);
    std::cout << "Calibrate gripper done" << std::endl;
    return 0;
}