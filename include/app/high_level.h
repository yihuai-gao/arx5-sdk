#ifndef HIGH_LEVEL_H
#define HIGH_LEVEL_H
#include "low_level.h"
#include "solver.h"
#include <thread>
#include <chrono>
#include <mutex>

struct HighState
{
    double timestamp = 0.0f;
    Vec6d pose_6d;                // x, y, z, roll, pitch, yaw
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}
    double gripper_torque = 0.0f; // Nm
    HighState() : pose_6d(Vec6d::Zero()) {}
    HighState(Vec6d pose_6d, double gripper_pos) : pose_6d(pose_6d), gripper_pos(gripper_pos) {}
    HighState operator+(const HighState &other) const
    {
        return HighState(pose_6d + other.pose_6d, gripper_pos + other.gripper_pos);
    }
    HighState operator*(const double &scalar) const
    {
        return HighState(pose_6d * scalar, gripper_pos * scalar);
    }
    Vec6d &get_pose_6d_ref()
    {
        return pose_6d;
    }
};

class Arx5HighLevel
{
public:
    Arx5HighLevel();
    ~Arx5HighLevel();

    void set_high_cmd(HighState new_cmd);
    std::tuple<HighState, HighState> get_high_cmd();
    std::tuple<LowState, LowState> get_low_cmd();
    HighState get_high_state();
    LowState get_low_state();
    double get_timestamp();
    void set_gain(Gain new_gain);
    Gain get_gain();

    void reset_to_home();
    void set_to_damping();

    void set_log_level(LogLevel log_level);

private:
    Arx5LowLevel _low_level;
    Arx5Solver _solver;
    HighState _input_high_cmd;
    HighState _output_high_cmd;
    HighState _high_state;
    LowState _low_state;
    double _clipping_output_threshold = 0.001;

    void _background_gravity_compensation_task();
    void _update_output_cmd();
    std::thread _background_gravity_compensation;
    bool _background_gravity_compensation_running = false;
    bool _destroy_background_threads = false;
    bool _enable_ee_vel_clipping = true;
    std::mutex _cmd_mutex;
    std::mutex _state_mutex;
    LogLevel _log_level = INFO;

    int _moving_window_size = 1; // 1 for no filtering
    double _LOOK_AHEAD_TIME = 0.1;
    MovingAverage6d _joint_pos_filter{_moving_window_size};
    MovingAverage6d _joint_torque_filter{_moving_window_size};
};

#endif