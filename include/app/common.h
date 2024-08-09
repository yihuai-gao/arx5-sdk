#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <string>
#include <thread>
#include <vector>

namespace arx
{
using Pose6d = Eigen::Matrix<double, 6, 1>;
using VecDoF = Eigen::VectorXd; // mutable size according to the robot motor number
enum class MotorType
{
    EC_A4310,
    DM_J4310,
    DM_J4340,
    None, // If any motor is disabled (e.g. gripper motor)
};

struct JointState
{
    double timestamp = 0.0f;
    VecDoF pos;
    VecDoF vel;
    VecDoF torque;
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}
    double gripper_torque = 0.0f; // Nm
    JointState(int dof) : pos(VecDoF::Zero(dof)), vel(VecDoF::Zero(dof)), torque(VecDoF::Zero(dof))
    {
    }
    JointState(VecDoF pos, VecDoF vel, VecDoF torque, double gripper_pos)
        : pos(pos), vel(vel), torque(torque), gripper_pos(gripper_pos)
    {
    }
    JointState operator+(const JointState &other) const
    {
        return JointState(pos + other.pos, vel + other.vel, torque + other.torque, gripper_pos + other.gripper_pos);
    }
    JointState operator*(const double &scalar) const
    {
        return JointState(pos * scalar, vel * scalar, torque * scalar, gripper_pos * scalar);
    }
    // For pybind11 to update values
    VecDoF &get_pos_ref()
    {
        return pos;
    }
    VecDoF &get_vel_ref()
    {
        return vel;
    }
    VecDoF &get_torque_ref()
    {
        return torque;
    }
};

struct Gain
{
    VecDoF kp;
    VecDoF kd;
    float gripper_kp = 0.0f;
    float gripper_kd = 0.0f;
    Gain(int dof) : kp(VecDoF::Zero(dof)), kd(VecDoF::Zero(dof))
    {
    }
    Gain(VecDoF kp, VecDoF kd, float gripper_kp, float gripper_kd)
        : kp(kp), kd(kd), gripper_kp(gripper_kp), gripper_kd(gripper_kd)
    {
    }
    Gain operator+(const Gain &other) const
    {
        return Gain(kp + other.kp, kd + other.kd, gripper_kp + other.gripper_kp, gripper_kd + other.gripper_kd);
    }
    Gain operator*(const double &scalar) const
    {
        return Gain(kp * scalar, kd * scalar, gripper_kp * scalar, gripper_kd * scalar);
    }
    VecDoF &get_kp_ref()
    {
        return kp;
    }
    VecDoF &get_kd_ref()
    {
        return kd;
    }
};

struct EEFState
{
    double timestamp = 0.0f;
    Pose6d pose_6d;               // x, y, z, roll, pitch, yaw
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}
    double gripper_torque = 0.0f; // Nm
    EEFState() : pose_6d(Pose6d::Zero())
    {
    }
    EEFState(Pose6d pose_6d, double gripper_pos) : pose_6d(pose_6d), gripper_pos(gripper_pos)
    {
    }
    EEFState operator+(const EEFState &other) const
    {
        return EEFState(pose_6d + other.pose_6d, gripper_pos + other.gripper_pos);
    }
    EEFState operator*(const double &scalar) const
    {
        return EEFState(pose_6d * scalar, gripper_pos * scalar);
    }
    Pose6d &get_pose_6d_ref()
    {
        return pose_6d;
    }
};

struct RobotConfig
{
    VecDoF joint_pos_min = (VecDoF(6) << -3.14, -0.05, -0.1, -1.6, -1.57, -2).finished();
    VecDoF joint_pos_max = (VecDoF(6) << 2.618, 3.14, 3.24, 1.55, 1.57, 2).finished();

    VecDoF joint_vel_max = (VecDoF(6) << 3.0, 2.0, 2.0, 2.0, 3.0, 3.0).finished();          // rad/s
    VecDoF joint_torque_max = (VecDoF(6) << 30.0, 40.0, 30.0, 15.0, 10.0, 10.0).finished(); // N*m
    VecDoF ee_vel_max = (VecDoF(6) << 0.6, 0.6, 0.6, 1.8, 1.8, 1.8).finished();
    // end effector speed: m/s for (x, y, z), rad/s for (roll, pitch, yaw)

    double gripper_vel_max = 0.1; // m/s
    double gripper_torque_max = 1.5;
    double gripper_width = 0.085; // fully opened: GRIPPER_WIDTH, fully closed: 0

    VecDoF default_kp = (VecDoF(6) << 80, 80, 80, 50, 40, 20).finished();
    VecDoF default_kd = (VecDoF(6) << 1.0, 1.0, 1.0, 1.0, 0.8, 1.0).finished();
    double default_gripper_kp = 30.0;
    double default_gripper_kd = 0.2;
    int over_current_cnt_max = 20; // 0.1s
    double gripper_open_readout = 4.8;

    int joint_dof = 6;

    std::array<int, 6> motor_id;
    std::string model;
    std::array<MotorType, 6> motor_type;
    int gripper_motor_id;
    MotorType gripper_motor_type;

    double controller_dt;

    // Will be used in inverse dynamics calculation.
    // Please change it to other values if the robot arm is not placed on the ground.
    Eigen::Vector3d gravity_vector = (Eigen::Vector3d() << 0, 0, -9.807).finished();

    std::string base_link_name = "base_link";
    std::string eef_link_name = "eef_link";

    RobotConfig(std::string model, double controller_dt) : model(model), controller_dt(controller_dt)
    {
        if (model == "X5")
        {
            motor_id = {1, 2, 4, 5, 6, 7};
            motor_type = {MotorType::EC_A4310, MotorType::EC_A4310, MotorType::EC_A4310,
                          MotorType::DM_J4310, MotorType::DM_J4310, MotorType::DM_J4310};
            gripper_motor_id = 8;
            gripper_motor_type = MotorType::DM_J4310;
        }
        else if (model == "L5")
        {
            motor_id = {1, 2, 4, 5, 6, 7};
            motor_type = {MotorType::DM_J4340, MotorType::DM_J4340, MotorType::DM_J4340,
                          MotorType::DM_J4310, MotorType::DM_J4310, MotorType::DM_J4310};
            gripper_motor_id = 8;
            gripper_motor_type = MotorType::DM_J4310;
        }
        else
        {
            throw std::invalid_argument("Robot model not supported: " + model + ". Only 'X5' and 'L5' available.");
        }
    }
};

} // namespace arx

#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us()                                                                                                  \
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()

#endif
