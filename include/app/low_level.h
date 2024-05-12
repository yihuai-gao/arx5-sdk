#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H
#include <stdlib.h>
#include "../hardware/arx_can.h"
#include "../hardware/motor.h"
#include "common.h"
#include "utils.h"
#include <array>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <chrono>
#include <mutex>

struct LowState
{
    double timestamp = 0.0f;
    Vec6d pos;
    Vec6d vel;
    Vec6d torque;
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}
    double gripper_torque = 0.0f; // Nm
    LowState() : pos(Vec6d::Zero()), vel(Vec6d::Zero()), torque(Vec6d::Zero()) {}
    LowState(Vec6d pos, Vec6d vel, Vec6d torque, double gripper_pos) : pos(pos), vel(vel), torque(torque), gripper_pos(gripper_pos) {}
    LowState operator+(const LowState &other) const
    {
        return LowState(pos + other.pos, vel + other.vel, torque + other.torque, gripper_pos + other.gripper_pos);
    }
    LowState operator*(const double &scalar) const
    {
        return LowState(pos * scalar, vel * scalar, torque * scalar, gripper_pos * scalar);
    }
    // For pybind11 to update values
    Vec6d &get_pos_ref()
    {
        return pos;
    }
    Vec6d &get_vel_ref()
    {
        return vel;
    }
    Vec6d &get_torque_ref()
    {
        return torque;
    }
};

struct Gain
{
    Vec6d kp;
    Vec6d kd;
    float gripper_kp = 0.0f;
    float gripper_kd = 0.0f;
    Gain() : kp(Vec6d::Zero()), kd(Vec6d::Zero()) {}
    Gain(Vec6d kp, Vec6d kd, float gripper_kp, float gripper_kd) : kp(kp), kd(kd), gripper_kp(gripper_kp), gripper_kd(gripper_kd) {}
    Gain operator+(const Gain &other) const
    {
        return Gain(kp + other.kp, kd + other.kd, gripper_kp + other.gripper_kp, gripper_kd + other.gripper_kd);
    }
    Gain operator*(const double &scalar) const
    {
        return Gain(kp * scalar, kd * scalar, gripper_kp * scalar, gripper_kd * scalar);
    }
    Vec6d &get_kp_ref()
    {
        return kp;
    }
    Vec6d &get_kd_ref()
    {
        return kd;
    }
};

class Arx5LowLevel
{
public:
    Arx5LowLevel();
    ~Arx5LowLevel();

    void send_recv_once();
    void enable_background_send_recv();
    void disable_background_send_recv();

    void set_low_cmd(LowState new_cmd);
    std::tuple<LowState, LowState> get_low_cmd();
    LowState get_state();

    void set_gain(Gain new_gain);
    Gain get_gain();

    Vec6d clip_joint_pos(Vec6d pos);
    void reset_to_home();
    void set_to_damping();

    void calibrate_gripper();
    void calibrate_joint(int joint_id);
    double get_timestamp();

    bool is_damping();

private:
    const double _GRIPPER_OPEN_READOUT = 4.8;
    void _background_send_recv_task();
    void _send_recv();
    void _check_current();
    int _over_current_cnt = 0;
    LowState _output_low_cmd;
    LowState _input_low_cmd;
    LowState _low_state;
    Gain _gain;
    can _can_handle;
    std::thread _background_send_recv;
    bool _background_send_recv_running = false;
    bool _destroy_background_threads = false;
    std::mutex _cmd_mutex;
    std::mutex _state_mutex;
    bool _enable_vel_clipping = true;
    bool _enable_torque_clipping = true;
    void _update_output_cmd();
    int _log_level = INFO;
    int _start_time_us;
    const std::array<int, 7> _MOTOR_ID = {1, 2, 4, 5, 6, 7, 8};
};

#endif