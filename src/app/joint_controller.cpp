#include "app/joint_controller.h"
#include <sys/syscall.h>
#include <sys/types.h>
#include <array>
#include <stdexcept>
#include "utils.h"
using namespace arx;

Arx5JointController::Arx5JointController(std::string model,
                                         std::string can_name)
    : _can_handle(can_name),
      _logger(spdlog::stdout_color_mt(model + std::string("_") + can_name)),
      _ROBOT_CONFIG(RobotConfig(model, _CONTROLLER_DT)) {

  _logger->set_pattern("[%H:%M:%S %n %^%l%$] %v");

  _init_robot();
  _background_send_recv_thread =
      std::thread(&Arx5JointController::_background_send_recv, this);
  _logger->info("Background send_recv task is running at ID: {}",
                syscall(SYS_gettid));
}

Arx5JointController::~Arx5JointController() {
  Gain damping_gain = Gain();
  damping_gain.kd = _ROBOT_CONFIG.default_kd;
  damping_gain.kd[0] *= 3;
  damping_gain.kd[1] *= 3;
  damping_gain.kd[2] *= 3;
  damping_gain.kd[3] *= 1.5;
  _logger->info("Set to damping before exit");
  set_gain(damping_gain);
  set_joint_cmd(JointState());
  _enable_gravity_compensation = false;
  sleep_ms(2000);
  _destroy_background_threads = true;
  _background_send_recv_thread.join();
  _logger->info("background send_recv task joined");
}

void Arx5JointController::_init_robot() {

  bool succeeded = true;
  for (int i = 0; i < 7; ++i) {
    if (_ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4310 ||
        _ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4340) {
      int id = _ROBOT_CONFIG.motor_id[i];
      succeeded = _can_handle.enable_DM_motor(id);
      if (!succeeded) {
        throw std::runtime_error("Failed to enable motor " +
                                 std::to_string(id) +
                                 ". Please check the connection.");
      }
      usleep(1000);
    }
  }

  Gain gain = Gain();
  gain.kd = _ROBOT_CONFIG.default_kd;
  set_joint_cmd(JointState());  // initialize joint command to zero
  set_gain(gain);               // set to damping by default
  for (int i = 0; i <= 10; ++i) {
    // make sure all the motor positions are updated
    succeeded = _send_recv();
    if (!succeeded) {
      throw std::runtime_error("Failed to send and receive motor command.");
    }
    sleep_ms(5);
  }
  // Check whether any motor has non-zero position
  if (_joint_state.pos == Vec6d::Zero()) {
    _logger->error(
        "All motors are not initialized. Please check the "
        "connection or power of the arm.");
    throw std::runtime_error(
        "All motors are not initialized. Please check the "
        "connection or power of the arm.");
  }
}

JointState Arx5JointController::get_state() {
  std::lock_guard<std::mutex> guard(_state_mutex);
  return _joint_state;
}

Vec6d Arx5JointController::get_tool_pose() {
  if (_solver == nullptr) {
    _logger->error("Solver is not initialized, cannot run kinematics.");
    return Vec6d::Zero();
  }
  return _solver->forward_kinematics(_joint_state.pos);
}

RobotConfig Arx5JointController::get_robot_config() {
  return _ROBOT_CONFIG;
}

void Arx5JointController::send_recv_once() {
  if (_background_send_recv_running) {
    std::cout << "send_recv task is already running in background. "
                 "send_recv_once is ignored."
              << std::endl;
    return;
  }
  _send_recv();
  _over_current_protection();
}

void Arx5JointController::_update_output_cmd() {
  std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);

  JointState prev_output_cmd = _output_joint_cmd;

  _output_joint_cmd = _input_joint_cmd;

  if (_enable_gravity_compensation && _solver != nullptr) {
    Vec6d gravity_torque = _solver->inverse_dynamics(
        _joint_state.pos, Vec6d::Zero(), Vec6d::Zero());
    _output_joint_cmd.torque += gravity_torque;
  }

  // Joint velocity clipping
  double dt = _CONTROLLER_DT;
  for (int i = 0; i < 6; ++i) {
    if (_gain.kp[i] > 0) {

      double delta_pos = _input_joint_cmd.pos[i] - prev_output_cmd.pos[i];
      double max_vel = _ROBOT_CONFIG.joint_vel_max[i];
      if (std::abs(delta_pos) > max_vel * dt) {
        _output_joint_cmd.pos[i] =
            prev_output_cmd.pos[i] +
            max_vel * dt * delta_pos / std::abs(delta_pos);
        _logger->debug("Joint {} pos {:.3f} pos cmd clipped: {:.3f} to {:.3f}",
                       i, _joint_state.pos[i], _input_joint_cmd.pos[i],
                       _output_joint_cmd.pos[i]);
      }
    } else {
      _output_joint_cmd.pos[i] = _joint_state.pos[i];
    }

    // Gripper pos clipping
    if (_gain.gripper_kp > 0) {
      double gripper_delta_pos =
          _input_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos;
      if (std::abs(gripper_delta_pos) / dt > _ROBOT_CONFIG.gripper_vel_max) {
        _output_joint_cmd.gripper_pos =
            prev_output_cmd.gripper_pos + _ROBOT_CONFIG.gripper_vel_max * dt *
                                              gripper_delta_pos /
                                              std::abs(gripper_delta_pos);
        if (std::abs(_input_joint_cmd.gripper_pos -
                     _output_joint_cmd.gripper_pos) >= 0.001)
          _logger->debug("Gripper pos cmd clipped: {:.3f} to {:.3f}",
                         _input_joint_cmd.gripper_pos,
                         _output_joint_cmd.gripper_pos);
      }
    } else {
      _output_joint_cmd.gripper_pos = _joint_state.gripper_pos;
    }
  }

  // Joint pos clipping
  for (int i = 0; i < 6; ++i) {
    if (_output_joint_cmd.pos[i] < _ROBOT_CONFIG.joint_pos_min[i]) {
      _logger->debug(
          "Joint {} pos {:.3f} pos cmd clipped from {:.3f} to min {:.3f}", i,
          _joint_state.pos[i], _output_joint_cmd.pos[i],
          _ROBOT_CONFIG.joint_pos_min[i]);
      _output_joint_cmd.pos[i] = _ROBOT_CONFIG.joint_pos_min[i];
    } else if (_output_joint_cmd.pos[i] > _ROBOT_CONFIG.joint_pos_max[i]) {
      _logger->debug(
          "Joint {} pos {:.3f} pos cmd clipped from {:.3f} to max {:.3f}", i,
          _joint_state.pos[i], _output_joint_cmd.pos[i],
          _ROBOT_CONFIG.joint_pos_max[i]);
      _output_joint_cmd.pos[i] = _ROBOT_CONFIG.joint_pos_max[i];
    }
  }
  // Gripper pos clipping
  if (_output_joint_cmd.gripper_pos < 0) {
    if (_output_joint_cmd.gripper_pos < -0.005)
      _logger->debug("Gripper pos cmd clipped from {:.3f} to min: {:.3f}",
                     _output_joint_cmd.gripper_pos, 0.0);
    _output_joint_cmd.gripper_pos = 0;
  } else if (_output_joint_cmd.gripper_pos > _ROBOT_CONFIG.gripper_width) {
    if (_output_joint_cmd.gripper_pos > _ROBOT_CONFIG.gripper_width + 0.005)
      _logger->debug("Gripper pos cmd clipped from {:.3f} to max: {:.3f}",
                     _output_joint_cmd.gripper_pos,
                     _ROBOT_CONFIG.gripper_width);
    _output_joint_cmd.gripper_pos = _ROBOT_CONFIG.gripper_width;
  }
  if (std::abs(_joint_state.gripper_torque) >
      _ROBOT_CONFIG.gripper_torque_max / 2) {
    double sign = _joint_state.gripper_torque > 0
                      ? 1
                      : -1;  // -1 for closing blocked, 1 for opening blocked
    double delta_pos =
        _output_joint_cmd.gripper_pos -
        prev_output_cmd
            .gripper_pos;  // negative for closing, positive for opening
    if (delta_pos * sign > 0) {
      _logger->debug(
          "Gripper torque is too large, gripper pos cmd is not updated");
      _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos;
    }
  }

  // Torque clipping
  for (int i = 0; i < 6; ++i) {
    if (_output_joint_cmd.torque[i] > _ROBOT_CONFIG.joint_torque_max[i]) {
      _logger->debug("Joint {} torque cmd clipped from {:.3f} to max {:.3f}", i,
                     _output_joint_cmd.torque[i],
                     _ROBOT_CONFIG.joint_torque_max[i]);
      _output_joint_cmd.torque[i] = _ROBOT_CONFIG.joint_torque_max[i];
    } else if (_output_joint_cmd.torque[i] <
               -_ROBOT_CONFIG.joint_torque_max[i]) {
      _logger->debug("Joint {} torque cmd clipped from {:.3f} to min {:.3f}", i,
                     _output_joint_cmd.torque[i],
                     -_ROBOT_CONFIG.joint_torque_max[i]);
      _output_joint_cmd.torque[i] = -_ROBOT_CONFIG.joint_torque_max[i];
    }
  }
}

double Arx5JointController::get_timestamp() {
  return double(get_time_us() - _start_time_us) / 1e6;
}

bool Arx5JointController::_send_recv() {
  // TODO: in the motor documentation, there shouldn't be these torque constants. Torque will go directly into the motors
  const double torque_constant_EC_A4310 = 1.4;  // Nm/A
  const double torque_constant_DM_J4310 = 0.424;
  const double torque_constant_DM_J4340 = 1.0;
  bool succeeded = true;
  int start_time_us = get_time_us();

  _update_output_cmd();
  int update_cmd_time_us = get_time_us();
  int communicate_sleep_us = 150;

  for (int i = 0; i < 6; i++) {
    int start_send_motor_time_us = get_time_us();
    if (_ROBOT_CONFIG.motor_type[i] == MotorType::EC_A4310) {
      succeeded = _can_handle.send_EC_motor_cmd(
          _ROBOT_CONFIG.motor_id[i], _gain.kp[i], _gain.kd[i],
          _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
          _output_joint_cmd.torque[i] / torque_constant_EC_A4310);
    } else if (_ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4310) {

      succeeded = _can_handle.send_DM_motor_cmd(
          _ROBOT_CONFIG.motor_id[i], _gain.kp[i], _gain.kd[i],
          _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
          _output_joint_cmd.torque[i] / torque_constant_DM_J4310);

    } else if (_ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4340) {
      succeeded = _can_handle.send_DM_motor_cmd(
          _ROBOT_CONFIG.motor_id[i], _gain.kp[i], _gain.kd[i],
          _output_joint_cmd.pos[i], _output_joint_cmd.vel[i],
          _output_joint_cmd.torque[i] / torque_constant_DM_J4340);
    } else {
      _logger->error("Motor type not supported.");
      return false;
    }
    int finish_send_motor_time_us = get_time_us();
    if (!succeeded) {
      _logger->error("Failed to send motor command to motor {}",
                     _ROBOT_CONFIG.motor_id[i]);
      return false;
    }
    sleep_us(communicate_sleep_us -
             (finish_send_motor_time_us - start_send_motor_time_us));
  }

  // Send gripper command (gripper is using DM motor)
  int start_send_motor_time_us = get_time_us();

  double gripper_motor_pos = _output_joint_cmd.gripper_pos /
                             _ROBOT_CONFIG.gripper_width *
                             _ROBOT_CONFIG.gripper_open_readout;
  succeeded =
      _can_handle.send_DM_motor_cmd(_ROBOT_CONFIG.motor_id[6], _gain.gripper_kp,
                                    _gain.gripper_kd, gripper_motor_pos, 0, 0);
  int finish_send_motor_time_us = get_time_us();

  if (!succeeded) {
    _logger->error("Failed to send motor command to motor {}",
                   _ROBOT_CONFIG.motor_id[6]);
    return false;
  }
  sleep_us(communicate_sleep_us -
           (finish_send_motor_time_us - start_send_motor_time_us));

  int start_get_motor_msg_time_us = get_time_us();
  std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
  int get_motor_msg_time_us = get_time_us();

  // _logger->trace("update_cmd: {} us, send_motor_0: {} us, send_motor_1: {} us, send_motor_2: {} us, send_motor_3: {} us, send_motor_4: {} us, send_motor_5: {} us, send_motor_6: {} us, get_motor_msg: {} us",
  //                update_cmd_time_us - start_time_us, send_motor_0_time_us - start_send_motor_0_time_us, send_motor_1_time_us - start_send_motor_1_time_us, send_motor_2_time_us - start_send_motor_2_time_us,
  //                send_motor_3_time_us - start_send_motor_3_time_us, send_motor_4_time_us - start_send_motor_4_time_us, send_motor_5_time_us - start_send_motor_5_time_us, send_motor_6_time_us - start_send_motor_6_time_us, get_motor_msg_time_us - start_get_motor_msg_time_us);

  std::lock_guard<std::mutex> guard_state(_state_mutex);
  std::array<int, 7> ids = _ROBOT_CONFIG.motor_id;

  _joint_state.pos[0] = motor_msg[0].angle_actual_rad;
  _joint_state.pos[1] = motor_msg[1].angle_actual_rad;
  _joint_state.pos[2] = motor_msg[3].angle_actual_rad;
  _joint_state.pos[3] = motor_msg[4].angle_actual_rad;
  _joint_state.pos[4] = motor_msg[5].angle_actual_rad;
  _joint_state.pos[5] = motor_msg[6].angle_actual_rad;
  _joint_state.gripper_pos = motor_msg[7].angle_actual_rad /
                             _ROBOT_CONFIG.gripper_open_readout *
                             _ROBOT_CONFIG.gripper_width;

  _joint_state.vel[0] = motor_msg[0].speed_actual_rad;
  _joint_state.vel[1] = motor_msg[1].speed_actual_rad;
  _joint_state.vel[2] = motor_msg[3].speed_actual_rad;
  _joint_state.vel[3] = motor_msg[4].speed_actual_rad;
  _joint_state.vel[4] = motor_msg[5].speed_actual_rad;
  _joint_state.vel[5] = motor_msg[6].speed_actual_rad;
  _joint_state.gripper_vel = motor_msg[7].speed_actual_rad /
                             _ROBOT_CONFIG.gripper_open_readout *
                             _ROBOT_CONFIG.gripper_width;

  // HACK: just to match the values (there must be something wrong)
  for (int i = 0; i < 6; i++) {
    if (_ROBOT_CONFIG.motor_type[i] == MotorType::EC_A4310) {
      _joint_state.torque[i] = motor_msg[ids[i]].current_actual_float *
                               torque_constant_EC_A4310 *
                               torque_constant_EC_A4310;
      // Why there are two torque_constant_EC_A4310?
    } else if (_ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4310) {
      _joint_state.torque[i] =
          motor_msg[ids[i]].current_actual_float * torque_constant_DM_J4310;
    } else if (_ROBOT_CONFIG.motor_type[i] == MotorType::DM_J4340) {
      _joint_state.torque[i] =
          motor_msg[ids[i]].current_actual_float * torque_constant_DM_J4340;
    }
  }

  _joint_state.gripper_torque =
      motor_msg[7].current_actual_float * torque_constant_DM_J4310;
  _joint_state.timestamp = get_timestamp();
  return true;
}

void Arx5JointController::_check_joint_state_sanity() {
  for (int i = 0; i < 6; ++i) {
    if (std::abs(_joint_state.pos[i]) > _ROBOT_CONFIG.joint_pos_max[i] + 3.14 ||
        std::abs(_joint_state.pos[i]) < _ROBOT_CONFIG.joint_pos_min[i] - 3.14) {
      _logger->error(
          "Joint {} pos data error: {:.3f}. Please restart the program.", i,
          _joint_state.pos[i]);
      _enter_emergency_state();
    }
    if (std::abs(_input_joint_cmd.pos[i]) >
            _ROBOT_CONFIG.joint_pos_max[i] + 3.14 ||
        std::abs(_input_joint_cmd.pos[i]) <
            _ROBOT_CONFIG.joint_pos_min[i] - 3.14) {
      _logger->error(
          "Joint {} command data error: {:.3f}. Please restart the program.", i,
          _input_joint_cmd.pos[i]);
      _enter_emergency_state();
    }
    if (std::abs(_joint_state.torque[i]) >
        100 * _ROBOT_CONFIG.joint_torque_max[i]) {
      _logger->error(
          "Joint {} torque data error: {:.3f}. Please restart the program.", i,
          _joint_state.torque[i]);
      _enter_emergency_state();
    }
  }
  // Gripper should be around 0~_ROBOT_CONFIG.gripper_width
  double gripper_width_tolerance = 0.005;  // m
  if (_joint_state.gripper_pos < -gripper_width_tolerance ||
      _joint_state.gripper_pos >
          _ROBOT_CONFIG.gripper_width + gripper_width_tolerance) {
    _logger->error(
        "Gripper position error: got {:.3f} but should be in 0~{:.3f} (m). "
        "Please close the gripper before turning the arm on or recalibrate "
        "gripper home and width.",
        _joint_state.gripper_pos, _ROBOT_CONFIG.gripper_width);
    _enter_emergency_state();
  }
}

void Arx5JointController::_over_current_protection() {
  bool over_current = false;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(_joint_state.torque[i]) > _ROBOT_CONFIG.joint_torque_max[i]) {
      over_current = true;
      _logger->error("Over current detected once on joint {}, current: {:.3f}",
                     i, _joint_state.torque[i]);
      break;
    }
  }
  if (std::abs(_joint_state.gripper_torque) >
      _ROBOT_CONFIG.gripper_torque_max) {
    over_current = true;
    _logger->error("Over current detected once on gripper, current: {:.3f}",
                   _joint_state.gripper_torque);
  }
  if (over_current) {
    _over_current_cnt++;
    if (_over_current_cnt > _ROBOT_CONFIG.over_current_cnt_max) {
      _logger->error(
          "Over current detected, robot is set to damping. Please restart the "
          "program.");
      _enter_emergency_state();
    }
  } else {
    _over_current_cnt = 0;
  }
}

void Arx5JointController::_enter_emergency_state() {
  Gain damping_gain;
  damping_gain.kd = _ROBOT_CONFIG.default_kd;
  damping_gain.kd[1] *= 3;
  damping_gain.kd[2] *= 3;
  damping_gain.kd[3] *= 1.5;
  set_gain(damping_gain);
  _input_joint_cmd.vel = Vec6d::Zero();
  _input_joint_cmd.torque = Vec6d::Zero();

  while (true) {
    std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);
    set_gain(damping_gain);
    _input_joint_cmd.vel = Vec6d::Zero();
    _input_joint_cmd.torque = Vec6d::Zero();
    _send_recv();
    sleep_ms(5);
  }
}

void Arx5JointController::_background_send_recv() {
  while (!_destroy_background_threads) {
    int start_time_us = get_time_us();
    if (_background_send_recv_running) {
      _over_current_protection();
      _check_joint_state_sanity();
      _send_recv();
    }
    int elapsed_time_us = get_time_us() - start_time_us;
    int sleep_time_us = int(_CONTROLLER_DT * 1e6) - elapsed_time_us;
    if (sleep_time_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
    } else if (sleep_time_us < -500) {
      _logger->debug(
          "Background send_recv task is running too slow, time: {} us",
          elapsed_time_us);
    }
  }
}

void Arx5JointController::set_joint_cmd(JointState new_cmd) {
  std::lock_guard<std::mutex> guard(_cmd_mutex);
  if (new_cmd.gripper_vel != 0 || new_cmd.gripper_torque != 0) {
    _logger->warn("Gripper vel and torque control is not supported yet.");
    new_cmd.gripper_vel = 0;
    new_cmd.gripper_torque = 0;
  }
  _input_joint_cmd = new_cmd;
}

std::tuple<JointState, JointState> Arx5JointController::get_joint_cmd() {
  std::lock_guard<std::mutex> guard(_cmd_mutex);
  return std::make_tuple(_input_joint_cmd, _output_joint_cmd);
}

Gain Arx5JointController::get_gain() {
  return _gain;
}

void Arx5JointController::set_gain(Gain new_gain) {

  // Make sure the robot doesn't jump when setting kp to non-zero
  if (_gain.kp.isZero() && !new_gain.kp.isZero()) {
    double max_pos_error =
        (_joint_state.pos - _output_joint_cmd.pos).cwiseAbs().maxCoeff();
    double error_threshold = 0.2;
    if (max_pos_error > error_threshold) {
      _logger->error(
          "Cannot set kp to non-zero when the joint pos cmd is far from "
          "current pos.");
      _logger->error("Current pos: {}, cmd pos: {}, threshold: {}",
                     vec2str(_joint_state.pos), vec2str(_output_joint_cmd.pos),
                     error_threshold);
      _background_send_recv_running = false;
      throw std::runtime_error(
          "Cannot set kp to non-zero when the joint pos cmd is far from "
          "current pos.");
    }
  }
  _gain = new_gain;
}

void Arx5JointController::reset_to_home() {
  JointState cmd;
  Gain gain;
  JointState init_state = get_state();
  Gain init_gain = get_gain();
  double init_gripper_kp = _gain.gripper_kp;
  double init_gripper_kd = _gain.gripper_kd;
  Gain target_gain =
      Gain(_ROBOT_CONFIG.default_kp, _ROBOT_CONFIG.default_kd,
           _ROBOT_CONFIG.default_gripper_kp, _ROBOT_CONFIG.default_gripper_kd);
  JointState target_state;
  if (init_state.pos == Vec6d::Zero()) {
    _logger->error(
        "Motor positions are not initialized. Please check the connection.");
    _background_send_recv_running = false;
    throw std::runtime_error(
        "Motor positions are not initialized. Please check the connection.");
  }
  // calculate the maximum joint position error
  double max_pos_error = (init_state.pos - Vec6d::Zero()).cwiseAbs().maxCoeff();
  max_pos_error = std::max(
      max_pos_error, init_state.gripper_pos * 2 / _ROBOT_CONFIG.gripper_width);
  // interpolate from current kp kd to default kp kd in max(max_pos_error, 0.5)s
  // and keep the target for max(max_pos_error, 0.5)s
  double step_num = std::max(max_pos_error, 0.5) / _CONTROLLER_DT;
  _logger->info("Start reset to home in {:.3f}s, max_pos_error: {:.3f}",
                std::max(max_pos_error, double(0.5)) + 0.5, max_pos_error);

  bool prev_running = _background_send_recv_running;
  _background_send_recv_running = true;
  for (int i = 0; i <= step_num; ++i) {
    double alpha = double(i) / step_num;
    gain = init_gain * (1 - alpha) + target_gain * alpha;
    cmd = init_state * (1 - alpha) + target_state * alpha;
    set_joint_cmd(cmd);
    sleep_ms(5);
    set_gain(gain);
  }
  // Hardcode 0.5 s
  sleep_ms(500);
  _logger->info("Finish reset to home");
  _background_send_recv_running = prev_running;
}

void Arx5JointController::set_to_damping() {
  JointState cmd;
  JointState state;
  Gain gain;
  JointState init_state = get_state();
  Gain init_gain = get_gain();
  Gain target_gain;
  target_gain.kd = _ROBOT_CONFIG.default_kd;
  _logger->info("Start set to damping");
  //  interpolate from current kp kd to default kp kd in 0.5s
  bool prev_running = _background_send_recv_running;
  int step_num = 20;  // 0.1s in total
  for (int i = 0; i <= step_num; ++i) {
    state = get_state();
    cmd.pos = state.pos;
    cmd.gripper_pos = state.gripper_pos;
    cmd.torque = Vec6d::Zero();
    cmd.vel = Vec6d::Zero();
    double alpha = double(i) / double(step_num);
    gain = init_gain * (1.0 - alpha) + target_gain * alpha;
    set_gain(gain);
    set_joint_cmd(cmd);
    sleep_ms(5);
  }
  sleep_ms(500);
  _logger->info("Finish set to damping");
  _background_send_recv_running = prev_running;
}

void Arx5JointController::calibrate_gripper() {
  bool prev_running = _background_send_recv_running;
  _background_send_recv_running = false;
  usleep(1000);
  for (int i = 0; i < 10; ++i) {
    _can_handle.send_DM_motor_cmd(8, 0, 0, 0, 0, 0);
    usleep(400);
  }
  _logger->info(
      "Start calibrating gripper. Please fully close the gripper and press "
      "enter to continue");
  std::cin.get();
  _can_handle.Set_Zero(0x08);
  usleep(400);
  for (int i = 0; i < 10; ++i) {
    _can_handle.send_DM_motor_cmd(8, 0, 0, 0, 0, 0);
    usleep(400);
  }
  usleep(400);
  _logger->info(
      "Finish setting zero point. Please fully open the gripper and press "
      "enter to continue");
  std::cin.get();

  for (int i = 0; i < 10; ++i) {
    _can_handle.send_DM_motor_cmd(8, 0, 0, 0, 0, 0);
    usleep(400);
  }
  std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
  std::cout << "Fully-open joint position readout: "
            << motor_msg[7].angle_actual_rad << std::endl;
  std::cout
      << "  Please update the _ROBOT_CONFIG.gripper_open_readout value in "
         "joint_controller.h to finish gripper calibration."
      << std::endl;
  if (prev_running) {
    _background_send_recv_running = true;
  }
}

void Arx5JointController::calibrate_joint(int joint_id) {
  bool prev_running = _background_send_recv_running;
  _background_send_recv_running = false;
  usleep(1000);
  int motor_id = _ROBOT_CONFIG.motor_id[joint_id];
  for (int i = 0; i < 10; ++i) {
    if (_ROBOT_CONFIG.motor_type[joint_id] == MotorType::EC_A4310)
      _can_handle.send_EC_motor_cmd(motor_id, 0, 0, 0, 0, 0);
    else
      _can_handle.send_DM_motor_cmd(motor_id, 0, 0, 0, 0, 0);
    usleep(400);
  }
  _logger->info(
      "Start calibrating joint {}. Please move the joint to the home position "
      "and press enter to continue",
      joint_id);
  std::cin.get();
  if (_ROBOT_CONFIG.motor_type[joint_id] == MotorType::EC_A4310)
    _can_handle.CAN_cmd_init(motor_id, 0x03);
  else
    _can_handle.Set_Zero(motor_id);
  usleep(400);
  for (int i = 0; i < 10; ++i) {
    if (_ROBOT_CONFIG.motor_type[joint_id] == MotorType::EC_A4310)
      _can_handle.send_EC_motor_cmd(motor_id, 0, 0, 0, 0, 0);
    else
      _can_handle.send_DM_motor_cmd(motor_id, 0, 0, 0, 0, 0);
    usleep(400);
  }
  usleep(400);
  _logger->info("Finish setting zero point for joint {}.", joint_id);
  if (prev_running) {
    _background_send_recv_running = true;
  }
}

void Arx5JointController::set_log_level(spdlog::level::level_enum log_level) {
  _logger->set_level(log_level);
}

void Arx5JointController::enable_gravity_compensation(std::string urdf_path) {
  _logger->info("Enable gravity compensation");
  _logger->info("Loading urdf from {}", urdf_path);
  _solver = std::make_shared<Arx5Solver>(urdf_path);
  _enable_gravity_compensation = true;
}

void Arx5JointController::disable_gravity_compensation() {
  _logger->info("Disable gravity compensation");
  _enable_gravity_compensation = false;
  _solver.reset();
}

void Arx5JointController::enable_background_send_recv() {
  _logger->info("Enable background send_recv");
  _background_send_recv_running = true;
}

void Arx5JointController::disable_background_send_recv() {
  _logger->info("Disable background send_recv");
  _background_send_recv_running = false;
}
