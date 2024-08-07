#ifndef CAN_H
#define CAN_H

#include "../libcan/SocketCAN.h"
#include "math_ops.h"
#include <array>
#include <iostream>
#include <mutex>
#include <stdint.h>
#include <string.h>

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

#define DM_P_MIN -12.5f // Radians
#define DM_P_MAX 12.5f
#define DM_V_MIN -45.0f // Rad/s
#define DM_V_MAX 45.0f
#define DM_KP_MIN 0.0f // N-m/rad
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f // N-m/rad/s
#define DM_KD_MAX 5.0f
#define DM_T_MIN -18.0f
#define DM_T_MAX 18.0f

typedef struct
{
    int16_t current_actual_int;
    float speed_actual_rad;
    float angle_actual_rad;
    uint16_t motor_id;
    uint8_t temperature;
    uint8_t error;
    float angle_actual_float;
    float current_actual_float;

    float gripper_pos;
    float gripper_spd;
    float gripper_cur;
    float gripper_last_pos;
    float gripper_totalangle;
    float round_cnt;

} OD_Motor_Msg;

void gripper_can_data_repack(uint32_t msgID, uint8_t *Data, std::array<OD_Motor_Msg, 10> &motor_msg);
void RV_can_data_repack(uint32_t msgID, uint8_t *Data, int32_t databufferlen, uint8_t comm_mode,
                        std::array<OD_Motor_Msg, 10> &motor_msg);
void DM_can_data_repack(uint8_t *Data, std::array<OD_Motor_Msg, 10> &motor_msg);

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