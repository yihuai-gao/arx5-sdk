#ifndef CAN_H
#define CAN_H

#include "../libcan/SocketCAN.h"
#include "motor.h"
#include <iostream>
#include <mutex>
#include <stdint.h>
#include <string.h>

class ArxCan
{
  public:
    ArxCan(std::string socket_name);
    ~ArxCan();

    void can_receive_frame(can_frame_t *frame);
    void can_cmd_init(uint16_t motor_id, uint8_t cmd);

    bool send_EC_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void set_motor(uint16_t motor_id, uint8_t cmd);

    bool send_DM_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    bool enable_DM_motor(uint16_t ID);
    void reset_zero_readout(uint16_t ID);
    void clear(uint16_t ID);
    bool transmit(can_frame_t &frame);
    const std::array<OD_Motor_Msg, 10> get_motor_msg();

  private:
    SocketCAN m_can_adapter;
    std::string m_socket_name;
    std::array<OD_Motor_Msg, 10> m_motor_msg;
    std::mutex m_motor_msg_mutex;
};

#endif