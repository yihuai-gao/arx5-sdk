#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H
#include <chrono>
#include <mutex>
#include <thread>
#include "app/joint_controller.h"
#include "app/solver.h"
namespace arx {

class Arx5CartesianController {
 public:
  Arx5CartesianController(std::string model, std::string can_name,
                          std::string urdf_path);
  ~Arx5CartesianController();

  void set_eef_cmd(EEFState new_cmd);
  std::tuple<EEFState, EEFState> get_eef_cmd();
  std::tuple<JointState, JointState> get_joint_cmd();
  EEFState get_eef_state();
  JointState get_joint_state();
  double get_timestamp();
  void set_gain(Gain new_gain);
  Gain get_gain();
  void set_log_level(spdlog::level::level_enum level);
  RobotConfig get_robot_config();

  void reset_to_home();
  Vec6d get_home_pose();
  void set_to_damping();

 private:
  const double _CONTROLLER_DT = 0.005;
  const RobotConfig _ROBOT_CONFIG;
  double _clipping_output_threshold = 0.001;
  int _moving_window_size = 1;  // 1 for no filtering

  EEFState _input_eef_cmd;
  EEFState _output_eef_cmd;
  JointState _input_joint_cmd;
  JointState _output_joint_cmd;
  JointState _joint_state;
  Gain _gain;

  ArxCan _can_handle;
  std::shared_ptr<spdlog::logger> _logger;
  std::thread _background_send_recv_thread;
  std::shared_ptr<Arx5Solver> _solver;

  void _init_robot();
  void _background_send_recv();
  void _update_output_cmd();
  void _calc_joint_cmd();
  void _over_current_protection();
  void _check_joint_state_sanity();
  void _enter_emergency_state();
  bool _send_recv();
  int _over_current_cnt = 0;
  bool _background_send_recv_running = false;
  bool _destroy_background_threads = false;
  bool _enable_gravity_compensation = true;
  std::mutex _cmd_mutex;
  std::mutex _state_mutex;
  int _start_time_us;

  MovingAverage6d _joint_pos_filter{_moving_window_size};
  MovingAverage6d _joint_torque_filter{_moving_window_size};
};

}  // namespace arx
#endif