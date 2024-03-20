#include "app/low_level.h"
#include <unistd.h>
#include <chrono>
#include <csignal>

Arx5LowLevel *arx5_low_level = new Arx5LowLevel();

void signal_handler(int signal)
{
    std::cout << "SIGINT received" << std::endl;
    delete arx5_low_level;
    exit(signal);
}

int main()
{
    std::signal(SIGINT, signal_handler);
    arx5_low_level->enable_background_send_recv();
    LowState cmd;
    int loop_cnt = 0;
    while (true)
    {
        LowState state = arx5_low_level->get_state();
        std::cout << "Gripper: " << state.gripper_pos << " Pos: " << state.pos[0] << " " << state.pos[1] << " " << state.pos[2] << " " << state.pos[3] << " " << state.pos[4] << " " << state.pos[5] << " " << std::endl;
        usleep(10000); // 10ms
        loop_cnt++;
        if (loop_cnt % 200 == 0)
        {
            arx5_low_level->reset_to_home();
            arx5_low_level->set_to_damping();
        }
    }
    return 0;
}