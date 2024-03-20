#ifndef COMMON_H
#define COMMON_H

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cstdarg>

using Vec6d = Eigen::Matrix<double, 6, 1>;

const Vec6d JOINT_POS_MIN = {-3.14, -0.005, -0.1, -1.671, -1.671, -1.57};
const Vec6d JOINT_POS_MAX = {2.618, 3.14, 3.24, 1.671, 1.671, 1.57};
const Vec6d DEFAULT_KP = {40, 40, 40, 40, 25, 10};
const Vec6d DEFAULT_KD = {5.0, 5.0, 3.0, 1.0, 0.8, 1.0};
const Vec6d JOINT_VEL_MAX = {3.0, 2.0, 2.0, 2.0, 3.0, 3.0};       // rad/s
const Vec6d JOINT_TORQUE_MAX = {12.0, 12.0, 12.0, 7.0, 5.0, 5.0}; // N*m
const Vec6d EE_VEL_MAX = {0.6, 0.6, 0.6, 1.8, 1.8, 1.8};          // end effector speed: m/s for (x, y, z), rad/s for (roll, pitch, yaw)
const std::vector<std::string> EE_POSE_NAMES = {"x", "y", "z", "roll", "pitch", "yaw"};
const double CTRL_DT = 0.005;
const double GRIPPER_VEL_MAX = 0.1; // m/s
const double GRIPPER_TORQUE_MAX = 0.8;
const double GRIPPER_WIDTH = 0.085; // fully opened: GRIPPER_WIDTH, fully closed: 0
const double DEFAULT_GRIPPER_KP = 30.0;
const double DEFAULT_GRIPPER_KD = 0.2;
const int OVER_CURRENT_CNT_MAX = 20; // 0.1s
#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

#endif
