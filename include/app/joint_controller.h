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

class Arx5JointController {
 public:
  Arx5JointController(std::string model, std::string can_name);
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
  double get_dt_s();
  RobotConfig get_robot_config();

  void set_log_level(spdlog::level::level_enum level);

 private:
  const RobotConfig _ROBOT_CONFIG;
  const double _CONTROLLER_DT = 0.002;
  void _background_send_recv();
  bool _send_recv();
  void _check_current();
  void _check_joint_state_sanity();
  void _enter_emergency_state();
  int _over_current_cnt = 0;
  JointState _output_joint_cmd;
  JointState _input_joint_cmd;
  JointState _joint_state;
  Gain _gain;
  ArxCan _can_handle;
  std::shared_ptr<spdlog::logger> _logger;
  std::thread _background_send_recv_thread;
  bool _background_send_recv_running = false;
  bool _destroy_background_threads = false;
  std::mutex _cmd_mutex;
  std::mutex _state_mutex;
  bool _enable_vel_clipping = true;
  bool _enable_torque_clipping = true;
  void _update_output_cmd();
  int _start_time_us;
  bool _enable_gravity_compensation = false;
  std::shared_ptr<Arx5Solver> _solver;
};

}  // namespace arx

#endif