#include "app/low_level.h"
#include <unistd.h>

int main()
{
    Arx5LowLevel arx5_low_level;
    int loop_cnt = 0;
    Gain gain;
    arx5_low_level.set_gain(gain);
    arx5_low_level.enable_background_send_recv();
    arx5_low_level.calibrate_joint(1);
    while (true)
    {
        LowState state = arx5_low_level.get_state();
        printf("Pos: %.3f %.3f %.3f %.3f %.3f %.3f\n", state.pos[0], state.pos[1], state.pos[2], state.pos[3], state.pos[4], state.gripper_pos);
        sleep_ms(100);
    }
    return 0;
}