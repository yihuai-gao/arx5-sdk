#ifndef MOTOR_H
#define MOTOR_H

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01

#include <stdint.h>
#include <array>
#include <iostream>
#include "math_ops.h"
typedef struct {
  uint16_t motor_id;
  uint8_t INS_code;   // instruction code.
  uint8_t motor_fbd;  // motor CAN communication feedback.
} MotorCommFbd;

typedef struct {
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

void MotorSetting(uint16_t motor_id, uint8_t cmd);
void MotorIDReading(uint8_t* pData, uint32_t* pCanID, uint8_t* pDataBufferLen);
void GetMotorParameter(uint16_t motor_id, uint8_t param_cmd, uint8_t* pData,
                       uint32_t* pCanID, uint8_t* pDataBufferLen);
void send_motor_ctrl_cmd(uint16_t motor_id, float kp, float kd, float pos,
                         float spd, float tor, uint8_t* Data, uint32_t* canID);
void send_motor_dm_cmd(uint16_t motor_id, float kp, float kd, float pos,
                       float spd, float tor, uint8_t* Data, uint32_t* canID);

void Gripper_can_data_repack(uint32_t msgID, uint8_t* Data,
                             std::array<OD_Motor_Msg, 10>& motor_msg);

void RV_can_data_repack(uint32_t msgID, uint8_t* Data, int32_t databufferlen,
                        uint8_t comm_mode,
                        std::array<OD_Motor_Msg, 10>& motor_msg);
void DM_can_data_repack(uint8_t* Data, std::array<OD_Motor_Msg, 10>& motor_msg);
void send_motor_position(uint16_t motor_id, float pos, uint16_t spd,
                         uint16_t cur, uint8_t ack_status, uint8_t* Data,
                         uint32_t* canID);

#endif
