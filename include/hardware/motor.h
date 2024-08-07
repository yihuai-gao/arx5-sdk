#ifndef MOTOR_H
#define MOTOR_H

#include "math_ops.h"
#include <array>
#include <iostream>
#include <stdint.h>

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

#endif
