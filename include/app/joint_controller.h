#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H
#include <stdlib.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>
#include "app/common.h"
#include "app/solver.h"
#include "hardware/arx_can.h"
#include "hardware/motor.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include "utils.h"

namespace arx {

struct JointState {
  double timestamp = 0.0f;
  Vec6d pos;
  Vec6d vel;
  Vec6d torque;
  double gripper_pos = 0.0f;     // m; 0 for close, GRIPPER_WIDTH for fully open
  double gripper_vel = 0.0f;     // s^{-1}
  double gripper_torque = 0.0f;  // Nm
  JointState()
      : pos(Vec6d::Zero()), vel(Vec6d::Zero()), torque(Vec6d::Zero()) {}
  JointState(Vec6d pos, Vec6d vel, Vec6d torque, double gripper_pos)
      : pos(pos), vel(vel), torque(torque), gripper_pos(gripper_pos) {}
  JointState operator+(const JointState& other) const {
    return JointState(pos + other.pos, vel + other.vel, torque + other.torque,
                      gripper_pos + other.gripper_pos);
  }
  JointState operator*(const double& scalar) const {
    return JointState(pos * scalar, vel * scalar, torque * scalar,
                      gripper_pos * scalar);
  }
  // For pybind11 to update values
  Vec6d& get_pos_ref() { return pos; }
  Vec6d& get_vel_ref() { return vel; }
  Vec6d& get_torque_ref() { return torque; }
};

struct Gain {
  Vec6d kp;
  Vec6d kd;
  float gripper_kp = 0.0f;
  float gripper_kd = 0.0f;
  Gain() : kp(Vec6d::Zero()), kd(Vec6d::Zero()) {}
  Gain(Vec6d kp, Vec6d kd, float gripper_kp, float gripper_kd)
      : kp(kp), kd(kd), gripper_kp(gripper_kp), gripper_kd(gripper_kd) {}
  Gain operator+(const Gain& other) const {
    return Gain(kp + other.kp, kd + other.kd, gripper_kp + other.gripper_kp,
                gripper_kd + other.gripper_kd);
  }
  Gain operator*(const double& scalar) const {
    return Gain(kp * scalar, kd * scalar, gripper_kp * scalar,
                gripper_kd * scalar);
  }
  Vec6d& get_kp_ref() { return kp; }
  Vec6d& get_kd_ref() { return kd; }
};

class Arx5JointController {
 public:
  Arx5JointController(std::string can_name);
  ~Arx5JointController();

  void send_recv_once();
  void enable_background_send_recv();
  void disable_background_send_recv();
  void enable_gravity_compensation(std::string urdf_path);
  void disable_gravity_compensation();

  void set_joint_cmd(JointState new_cmd);
  std::tuple<JointState, JointState> get_joint_cmd();
  JointState get_state();
  Vec6d get_tool_pose();

  void set_gain(Gain new_gain);
  Gain get_gain();

  Vec6d clip_joint_pos(Vec6d pos);
  void reset_to_home();
  void set_to_damping();

  void calibrate_gripper();
  void calibrate_joint(int joint_id);
  double get_timestamp();

  void set_log_level(spdlog::level::level_enum level);

  bool is_damping();

 private:
  const double _GRIPPER_OPEN_READOUT = 4.8;
  void _background_send_recv_task();
  void _send_recv();
  void _check_current();
  int _over_current_cnt = 0;
  JointState _output_joint_cmd;
  JointState _input_joint_cmd;
  JointState _joint_state;
  Gain _gain;
  ArxCan _can_handle;
  std::shared_ptr<spdlog::logger> _logger;
  std::thread _background_send_recv;
  bool _background_send_recv_running = false;
  bool _destroy_background_threads = false;
  std::mutex _cmd_mutex;
  std::mutex _state_mutex;
  bool _enable_vel_clipping = true;
  bool _enable_torque_clipping = true;
  void _update_output_cmd();
  int _start_time_us;
  const std::array<int, 7> _MOTOR_ID = {1, 2, 4, 5, 6, 7, 8};
  bool _enable_gravity_compensation = false;
  std::shared_ptr<Arx5Solver> _solver;
};

}  // namespace arx

#endif