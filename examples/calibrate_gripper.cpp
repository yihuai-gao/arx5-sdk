#include "app/low_level.h"
#include <unistd.h>

void test_gripper_movement(Arx5LowLevel &arx5_low_level)
{
    arx5_low_level.reset_to_home();
    LowState cmd;
    // Open the gripper in 1s
    int step_num = 400;
    double current_gripper_pos = arx5_low_level.get_state().gripper_pos;
    arx5_low_level.enable_background_send_recv();
    for (int i = 0; i <= step_num; ++i)
    {
        float alpha = double(i) / double(step_num);
        cmd.gripper_pos = current_gripper_pos * (1 - alpha) + GRIPPER_WIDTH * alpha;
        arx5_low_level.set_low_cmd(cmd);
        usleep(5000);
    }
    usleep(500000); // Hardcode 0.5 s
    // Close the gripper in 1s
    for (int i = 0; i <= step_num; ++i)
    {
        float alpha = double(i) / double(step_num);
        cmd.gripper_pos = GRIPPER_WIDTH * (1 - alpha) + 0 * alpha;
        arx5_low_level.set_low_cmd(cmd);
        usleep(5000);
    }
    usleep(500000); // Hardcode 0.5 s
    arx5_low_level.set_to_damping();
    arx5_low_level.disable_background_send_recv();
}

int main()
{
    Arx5LowLevel arx5_low_level;
    int loop_cnt = 0;
    arx5_low_level.calibrate_gripper();
    // test_gripper_movement(arx5_low_level);
    std::cout << "Calibrate gripper done" << std::endl;
    return 0;
}