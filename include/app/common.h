#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <string>
#include <thread>
#include <vector>

namespace arx {
using Vec6d = Eigen::Matrix<double, 6, 1>;

enum class MotorType {
  EC,
  DM,
};

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

struct EEFState {
  double timestamp = 0.0f;
  Vec6d pose_6d;                 // x, y, z, roll, pitch, yaw
  double gripper_pos = 0.0f;     // m; 0 for close, GRIPPER_WIDTH for fully open
  double gripper_vel = 0.0f;     // s^{-1}
  double gripper_torque = 0.0f;  // Nm
  EEFState() : pose_6d(Vec6d::Zero()) {}
  EEFState(Vec6d pose_6d, double gripper_pos)
      : pose_6d(pose_6d), gripper_pos(gripper_pos) {}
  EEFState operator+(const EEFState& other) const {
    return EEFState(pose_6d + other.pose_6d, gripper_pos + other.gripper_pos);
  }
  EEFState operator*(const double& scalar) const {
    return EEFState(pose_6d * scalar, gripper_pos * scalar);
  }
  Vec6d& get_pose_6d_ref() { return pose_6d; }

  const std::vector<std::string> EE_POSE_NAMES = {"x",    "y",     "z",
                                                  "roll", "pitch", "yaw"};
};

struct RobotConfig {
  Vec6d joint_pos_min =
      (Vec6d() << -3.14, -0.05, -0.1, -1.6, -1.57, -2).finished();
  Vec6d joint_pos_max =
      (Vec6d() << 2.618, 3.14, 3.24, 1.55, 1.57, 2).finished();
  Vec6d default_kp = (Vec6d() << 80, 80, 80, 50, 40, 20).finished();
  Vec6d default_kd = (Vec6d() << 1.0, 1.0, 1.0, 1.0, 0.8, 1.0).finished();
  Vec6d joint_vel_max =
      (Vec6d() << 3.0, 2.0, 2.0, 2.0, 3.0, 3.0).finished();  // rad/s
  Vec6d joint_torque_max =
      (Vec6d() << 30.0, 40.0, 30.0, 15.0, 10.0, 10.0).finished();  // N*m
  Vec6d ee_vel_max = (Vec6d() << 0.6, 0.6, 0.6, 1.8, 1.8, 1.8).finished();
  // end effector speed: m/s for (x, y, z), rad/s for (roll, pitch, yaw)

  double gripper_vel_max = 0.1;  // m/s
  double gripper_torque_max = 1.5;
  double gripper_width = 0.085;  // fully opened: GRIPPER_WIDTH, fully closed: 0

  double default_gripper_kp = 30.0;
  double default_gripper_kd = 0.2;
  int over_current_cnt_max = 20;  // 0.1s
  double gripper_open_readout = 4.8;

  int joint_dof = 6;

  std::array<int, 7> motor_id;
  std::string model;
  std::array<MotorType, 7> motor_type;

  RobotConfig(std::string model) : model(model) {
    if (model == "X5") {
      motor_id = {1, 2, 4, 5, 6, 7, 8};
      motor_type = {MotorType::EC, MotorType::EC, MotorType::EC, MotorType::DM,
                    MotorType::DM, MotorType::DM, MotorType::DM};
    } else if (model == "L5") {
      motor_id = {1, 2, 4, 5, 6, 7, 8};
      motor_type = {MotorType::DM, MotorType::DM, MotorType::DM, MotorType::DM,
                    MotorType::DM, MotorType::DM, MotorType::DM};
    } else {
      throw std::invalid_argument("Robot model not supported: " + model);
    }
  }
};

}  // namespace arx

#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us()                                      \
  std::chrono::duration_cast<std::chrono::microseconds>(   \
      std::chrono::steady_clock::now().time_since_epoch()) \
      .count()

#endif
