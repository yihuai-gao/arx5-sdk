#include "hardware/motor.h"
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f // T代表力矩
#define T_MAX 30.0f
#define I_MIN -30.0f // I代表电流
#define I_MAX 30.0f

#define DM_P_MIN -12.5f // Radians
#define DM_P_MAX 12.5f
#define DM_V_MIN -45.0f // Rad/s
#define DM_V_MAX 45.0f
#define DM_KP_MIN 0.0f // N-m/rad
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f // N-m/rad/s
#define DM_KD_MAX 5.0f
#define DM_T_MIN -18.0f // T代表力矩
#define DM_T_MAX 18.0f

// 格式转换
union RV_TypeConvert1
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert1;

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

MotorCommFbd motor_comm_fbd = {0};
OD_Motor_Msg rv_motor_msg[10] = {};
float magic_pos[3] = {0, 0, 0};
float magic_angle[3] = {0, 0, 0};
int t_magic_pos[3] = {0, 0, 0};
int t_magic_angle[3] = {0, 0, 0};
int magic_switch[2] = {0, 0};

void MotorIDReading(uint8_t *pData, uint32_t *pCanID, uint8_t *pDataBufferLen)
{
    *pCanID = 0x7FF;
    *pDataBufferLen = 4;
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0x00;
    pData[3] = 0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void GetMotorParameter(uint16_t motor_id, uint8_t param_cmd, uint8_t *pData, uint32_t *pCanID, uint8_t *pDataBufferLen)
{
    *pCanID = motor_id;
    *pDataBufferLen = 2;
    pData[0] = 0xE0;
    pData[1] = param_cmd;
}

// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(uint16_t motor_id, uint8_t cmd, uint8_t *Data, uint32_t *canID)
{
    *canID = motor_id;
    if (cmd == 0)
        return;
    Data[0] = motor_id >> 8;
    Data[1] = motor_id & 0xff;
    Data[2] = 0x00;
    Data[3] = cmd;
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, uint8_t *Data, uint32_t *canID)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    *canID = motor_id;

    if (kp > KP_MAX)
    {
        kp = KP_MAX;
    }
    else if (kp < KP_MIN)
    {
        kp = KP_MIN;
    }
    if (kd > KD_MAX)
    {
        kd = KD_MAX;
    }
    else if (kd < KD_MIN)
    {
        kd = KD_MIN;
    }
    if (pos > POS_MAX)
    {
        pos = POS_MAX;
    }
    else if (pos < POS_MIN)
    {
        pos = POS_MIN;
    }
    if (spd > SPD_MAX)
    {
        spd = SPD_MAX;
    }
    else if (spd < SPD_MIN)
    {
        spd = SPD_MIN;
    }
    if (tor > T_MAX)
    {
        tor = T_MAX;
    }
    else if (tor < T_MIN)
    {
        tor = T_MIN;
    }

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    Data[0] = 0x00 | (kp_int >> 7);                             // kp5
    Data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    Data[2] = kd_int & 0xFF;
    Data[3] = pos_int >> 8;
    Data[4] = pos_int & 0xFF;
    Data[5] = spd_int >> 4;
    Data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    Data[7] = tor_int & 0xff;
    // ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>send_motor_ctrl_cmd>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
}

void send_motor_dm_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, uint8_t *Data, uint32_t *canID)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    *canID = motor_id;

    if (kp > DM_KP_MAX)
    {
        kp = DM_KP_MAX;
    }
    else if (kp < DM_KP_MIN)
    {
        kp = DM_KP_MIN;
    }
    if (kd > DM_KD_MAX)
    {
        kd = DM_KD_MAX;
    }
    else if (kd < DM_KD_MIN)
    {
        kd = DM_KD_MIN;
    }
    if (pos > DM_P_MAX)
    {
        pos = DM_P_MAX;
    }
    else if (pos < DM_P_MIN)
    {
        pos = DM_P_MIN;
    }
    if (spd > DM_V_MAX)
    {
        spd = DM_V_MAX;
    }
    else if (spd < DM_V_MIN)
    {
        spd = DM_V_MIN;
    }
    if (tor > DM_T_MAX)
    {
        tor = DM_T_MAX;
    }
    else if (tor < DM_T_MIN)
    {
        tor = DM_T_MIN;
    }

    pos_tmp = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(spd, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(tor, DM_T_MIN, DM_T_MAX, 12);

    Data[0] = (pos_tmp >> 8);
    Data[1] = (pos_tmp & 0xFF);
    Data[2] = (vel_tmp >> 4);
    Data[3] = (((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    Data[4] = (kp_tmp & 0xFF);
    Data[5] = (kd_tmp >> 4);
    Data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    Data[7] = (tor_tmp & 0xFF);
}

void send_motor_position(uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status, uint8_t *Data, uint32_t *canID)
{
    *canID = motor_id;
    if (spd > 18000)
        spd = 18000;
    if (cur > 3000)
        cur = 3000;
    if (ack_status > 3)
        return;
    // ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>send_motor_ctrl_cmd>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    rv_type_convert1.to_float = pos;
    Data[0] = 0x20 | (rv_type_convert1.buf[3] >> 3);
    Data[1] = (rv_type_convert1.buf[3] << 5) | (rv_type_convert1.buf[2] >> 3);
    Data[2] = (rv_type_convert1.buf[2] << 5) | (rv_type_convert1.buf[1] >> 3);
    Data[3] = (rv_type_convert1.buf[1] << 5) | (rv_type_convert1.buf[0] >> 3);
    Data[4] = (rv_type_convert1.buf[0] << 5) | (spd >> 10);
    Data[5] = (spd & 0x3FC) >> 2;
    Data[6] = (spd & 0x03) << 6 | (cur >> 6);
    Data[7] = (cur & 0x3F) << 2 | ack_status;
}

void RV_can_data_repack(uint32_t msgID, uint8_t *Data, int32_t databufferlen, uint8_t comm_mode)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    if (comm_mode == 0x00) // Response mode
    {
        ack_status = Data[0] >> 5;
        motor_id_t = msgID - 1;
        rv_motor_msg[motor_id_t].motor_id = motor_id_t;
        rv_motor_msg[motor_id_t].error = Data[0] & 0x1F;
        if (ack_status == 1) // response frame 1
        {
            pos_int = Data[1] << 8 | Data[2];
            spd_int = Data[3] << 4 | (Data[4] & 0xF0) >> 4;
            cur_int = (Data[4] & 0x0F) << 8 | Data[5];

            rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
            rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
            rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
            rv_motor_msg[motor_id_t].temperature = (Data[6] - 50) / 2;
        }
        else if (ack_status == 2) // response frame 2
        {
            rv_type_convert.buf[0] = Data[4];
            rv_type_convert.buf[1] = Data[3];
            rv_type_convert.buf[2] = Data[2];
            rv_type_convert.buf[3] = Data[1];
            rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
            rv_motor_msg[motor_id_t].current_actual_int = Data[5] << 8 | Data[6];
            rv_motor_msg[motor_id_t].temperature = (Data[7] - 50) / 2;
            rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
        }
    }
}

void Swicth_DM_Moto(uint32_t msgID, uint8_t *Data)
{
    switch (Data[0])
    {
    case 0x05:
    case 0x06:
    case 0x07:
    {
        DM_can_data_repack(Data);
        break;
    }
    }
}
void DM_can_data_repack(uint8_t *Data)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int, spd_int, cur_int;

    // motor_id_t = Data[0] - 1;
    motor_id_t = (Data[0] & 0x0F) - 1;
    rv_motor_msg[motor_id_t].motor_id = motor_id_t;
    pos_int = (Data[1] << 8) | (Data[2]);
    spd_int = (Data[3] << 4) | (Data[4] >> 4);
    cur_int = ((Data[4] & 0x0F) << 8) | (Data[5]);

    rv_motor_msg[motor_id_t].angle_actual_rad = (float)uint_to_float(pos_int, -12.5, 12.5, 16);
    rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, -45.0, 45.0, 12);
    rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, -18.0, 18.0, 12);
    // ROS_WARN("moto 7 >> %f",rv_motor_msg[motor_id_t].angle_actual_rad);
}

void Gripper_can_data_repack(uint32_t msgID, uint8_t *Data)
{
    rv_motor_msg[8].gripper_last_pos = rv_motor_msg[8].gripper_pos;
    rv_motor_msg[8].gripper_spd = ((int16_t)(Data[5] << 8) | (Data[4])) / 57.3f;
    rv_motor_msg[8].gripper_pos = ((int16_t)(Data[7] << 8) | (Data[6])) + 32768; //	32768
    rv_motor_msg[8].gripper_cur = ((int16_t)(Data[2] << 8) | (Data[3]));
    if (rv_motor_msg[8].gripper_pos - rv_motor_msg[8].gripper_last_pos > 32768)
        rv_motor_msg[8].round_cnt--;
    else if (rv_motor_msg[8].gripper_pos - rv_motor_msg[8].gripper_last_pos < -32768)
        rv_motor_msg[8].round_cnt++;
    rv_motor_msg[8].gripper_totalangle = rv_motor_msg[8].round_cnt * 65536 + rv_motor_msg[8].gripper_pos;
}

void magic_pos_repack(uint32_t msgID, uint8_t *Data)
{
    t_magic_pos[0] = (int)((Data[0] << 8) | (Data[1]));
    t_magic_pos[1] = (int)((Data[2] << 8) | (Data[3]));
    t_magic_pos[2] = (int)((Data[4] << 8) | (Data[5]));

    magic_pos[0] = int_to_float(t_magic_pos[0], 1.0f, -1.0f, 16);
    magic_pos[1] = int_to_float(t_magic_pos[1], 1.0f, -1.0f, 16);
    magic_pos[2] = int_to_float(t_magic_pos[2], 1.0f, -1.0f, 16);
    // 输出 t_magic_pos[0]
    //  ROS_INFO("pos_x%f,pos_y%f,pos_z%f",magic_pos[0],magic_pos[1],magic_pos[2]);
    //  ROS_INFO("pos_x%d,pos_y%d,pos_z%d",t_magic_pos[0],t_magic_pos[1],t_magic_pos[2]);
}

void magic_angle_repack(uint32_t msgID, uint8_t *Data)
{
    t_magic_angle[0] = (int)((Data[0] << 8) | (Data[1]));
    t_magic_angle[1] = (int)((Data[2] << 8) | (Data[3]));
    t_magic_angle[2] = (int)((Data[4] << 8) | (Data[5]));

    magic_angle[0] = int_to_float(t_magic_angle[0], 4.0f, -4.0f, 16);
    magic_angle[1] = int_to_float(t_magic_angle[1], 4.0f, -4.0f, 16);
    magic_angle[2] = int_to_float(t_magic_angle[2], 4.0f, -4.0f, 16);
    // 输出 t_magic_pos[0]
    //  ROS_INFO("pit%f,yaw%f,roll%f",magic_angle[0],magic_angle[1],magic_angle[2]);
    //  ROS_INFO("pos_x%d,pos_y%d,pos_z%d",t_magic_pos[0],t_magic_pos[1],t_magic_pos[2]);
}

void magic_switch_repack(uint32_t msgID, uint8_t *Data)
{
    magic_switch[0] = (int)((Data[0] << 8) | (Data[1]));
    magic_switch[1] = (int)((Data[2] << 8) | (Data[3]));

    // 输出 t_magic_pos[0]
    //  ROS_INFO("magic_switch 1>>%d,2>>%d",magic_switch[0],magic_switch[1]);
    //  ROS_INFO("pos_x%d,pos_y%d,pos_z%d",t_magic_pos[0],t_magic_pos[1],t_magic_pos[2]);
}