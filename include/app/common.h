#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <eigen3/Eigen/Dense>
#include <string>
#include <thread>
#include <vector>

// namespace arx {

using Vec6d = Eigen::Matrix<double, 6, 1>;
const Vec6d JOINT_POS_MIN =
    (Vec6d() << -3.14, -0.005, -0.1, -1.6, -1.57, -2).finished();
const Vec6d JOINT_POS_MAX =
    (Vec6d() << 2.618, 3.14, 3.24, 1.55, 1.57, 2).finished();
const Vec6d DEFAULT_KP = (Vec6d() << 80, 80, 80, 50, 40, 20).finished();
const Vec6d DEFAULT_KD = (Vec6d() << 1.0, 1.0, 1.0, 1.0, 0.8, 1.0).finished();
const Vec6d JOINT_VEL_MAX =
    (Vec6d() << 3.0, 2.0, 2.0, 2.0, 3.0, 3.0).finished();  // rad/s
const Vec6d JOINT_TORQUE_MAX =
    (Vec6d() << 30.0, 40.0, 30.0, 15.0, 10.0, 10.0).finished();  // N*m
const Vec6d EE_VEL_MAX =
    (Vec6d() << 0.6, 0.6, 0.6, 1.8, 1.8, 1.8)
        .finished();  // end effector speed: m/s for (x, y, z), rad/s for (roll, pitch, yaw)
const std::vector<std::string> EE_POSE_NAMES = {"x",    "y",     "z",
                                                "roll", "pitch", "yaw"};
const double JOINT_CONTROLLER_DT = 0.002;
const double HIGH_LEVEL_DT = 0.005;
const double GRIPPER_VEL_MAX = 0.1;  // m/s
const double GRIPPER_TORQUE_MAX = 1.5;
const double GRIPPER_WIDTH =
    0.085;  // fully opened: GRIPPER_WIDTH, fully closed: 0
const double DEFAULT_GRIPPER_KP = 30.0;
const double DEFAULT_GRIPPER_KD = 0.2;
const int OVER_CURRENT_CNT_MAX = 20;  // 0.1s

enum class MotorType {
  EC,
  DM,
};
// }  // namespace arx

#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us()                                      \
  std::chrono::duration_cast<std::chrono::microseconds>(   \
      std::chrono::steady_clock::now().time_since_epoch()) \
      .count()

#endif
