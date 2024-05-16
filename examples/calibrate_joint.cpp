#include "app/joint_controller.h"
#include <unistd.h>

int main()
{
    Arx5JointController arx5_joint_controller;
    int loop_cnt = 0;
    Gain gain;
    arx5_joint_controller.set_gain(gain);
    arx5_joint_controller.enable_background_send_recv();
    arx5_joint_controller.calibrate_joint(1);
    while (true)
    {
        JointState state = arx5_joint_controller.get_state();
        printf("Pos: %.3f %.3f %.3f %.3f %.3f %.3f\n", state.pos[0], state.pos[1], state.pos[2], state.pos[3], state.pos[4], state.gripper_pos);
        sleep_ms(100);
    }
    return 0;
}