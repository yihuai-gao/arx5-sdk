#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "app/low_level.h"
#include "app/high_level.h"
#include "utils.h"
namespace py = pybind11;
using Vec6d = Eigen::Matrix<double, 6, 1>;
PYBIND11_MODULE(arx5_interface, m)
{
    m.attr("JOINT_POS_MIN") = JOINT_POS_MIN;
    m.attr("JOINT_POS_MAX") = JOINT_POS_MAX;
    m.attr("DEFAULT_KP") = DEFAULT_KP;
    m.attr("DEFAULT_KD") = DEFAULT_KD;
    m.attr("DEFAULT_GRIPPER_KP") = DEFAULT_GRIPPER_KP;
    m.attr("DEFAULT_GRIPPER_KD") = DEFAULT_GRIPPER_KD;
    m.attr("JOINT_VEL_MAX") = JOINT_VEL_MAX;
    m.attr("JOINT_TORQUE_MAX") = JOINT_TORQUE_MAX;
    m.attr("EE_VEL_MAX") = EE_VEL_MAX;
    m.attr("GRIPPER_VEL_MAX") = GRIPPER_VEL_MAX;
    m.attr("GRIPPER_WIDTH") = GRIPPER_WIDTH;
    m.attr("CTRL_DT") = CTRL_DT;
    py::enum_<LogLevel>(m, "LogLevel")
        .value("DEBUG", LogLevel::DEBUG)
        .value("INFO", LogLevel::INFO)
        .value("WARNING", LogLevel::WARNING)
        .value("ERROR", LogLevel::ERROR)
        .export_values();
    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def(py::init<Vec6d, Vec6d, Vec6d, double>())
        .def_readwrite("timestamp", &LowState::timestamp)
        .def_readwrite("gripper_pos", &LowState::gripper_pos)
        .def_readwrite("gripper_vel", &LowState::gripper_vel)
        .def_readwrite("gripper_torque", &LowState::gripper_torque)
        .def("__add__", [](const LowState &self, const LowState &other)
             { return self + other; })
        .def("__mul__", [](const LowState &self, const float &scalar)
             { return self * scalar; })
        .def("pos", &LowState::get_pos_ref, py::return_value_policy::reference)
        .def("vel", &LowState::get_vel_ref, py::return_value_policy::reference)
        .def("torque", &LowState::get_torque_ref, py::return_value_policy::reference);
    py::class_<Gain>(m, "Gain")
        .def(py::init<>())
        .def(py::init<Vec6d, Vec6d, double, double>())
        .def_readwrite("gripper_kp", &Gain::gripper_kp)
        .def_readwrite("gripper_kd", &Gain::gripper_kd)
        .def("__add__", [](const Gain &self, const Gain &other)
             { return self + other; })
        .def("__mul__", [](const Gain &self, const float &scalar)
             { return self * scalar; })
        .def("kp", &Gain::get_kp_ref, py::return_value_policy::reference)
        .def("kd", &Gain::get_kd_ref, py::return_value_policy::reference);
    py::class_<Arx5LowLevel>(m, "Arx5LowLevel")
        .def(py::init<>())
        .def("send_recv_once", &Arx5LowLevel::send_recv_once)
        .def("enable_background_send_recv", &Arx5LowLevel::enable_background_send_recv)
        .def("disable_background_send_recv", &Arx5LowLevel::disable_background_send_recv)
        .def("get_state", &Arx5LowLevel::get_state)
        .def("get_timestamp", &Arx5LowLevel::get_timestamp)
        .def("set_low_cmd", &Arx5LowLevel::set_low_cmd)
        .def("get_low_cmd", &Arx5LowLevel::get_low_cmd)
        .def("set_gain", &Arx5LowLevel::set_gain)
        .def("get_gain", &Arx5LowLevel::get_gain)
        .def("clip_joint_pos", &Arx5LowLevel::clip_joint_pos)
        .def("reset_to_home", &Arx5LowLevel::reset_to_home)
        .def("set_to_damping", &Arx5LowLevel::set_to_damping)
        .def("calibrate_gripper", &Arx5LowLevel::calibrate_gripper)
        .def("set_log_level", &Arx5LowLevel::set_log_level);
    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def(py::init<Vec6d, double>())
        .def_readwrite("timestamp", &HighState::timestamp)
        .def_readwrite("gripper_pos", &HighState::gripper_pos)
        .def_readwrite("gripper_vel", &HighState::gripper_vel)
        .def_readwrite("gripper_torque", &HighState::gripper_torque)
        .def("__add__", [](const HighState &self, const HighState &other)
             { return self + other; })
        .def("__mul__", [](const HighState &self, const float &scalar)
             { return self * scalar; })
        .def("pose_6d", &HighState::get_pose_6d_ref, py::return_value_policy::reference);
    py::class_<Arx5HighLevel>(m, "Arx5HighLevel")
        .def(py::init<>())
        .def("set_high_cmd", &Arx5HighLevel::set_high_cmd)
        .def("get_high_state", &Arx5HighLevel::get_high_state)
        .def("get_low_state", &Arx5HighLevel::get_low_state)
        .def("get_high_cmd", &Arx5HighLevel::get_high_cmd)
        .def("get_low_cmd", &Arx5HighLevel::get_low_cmd)
        .def("get_timestamp", &Arx5HighLevel::get_timestamp)
        .def("set_gain", &Arx5HighLevel::set_gain)
        .def("get_gain", &Arx5HighLevel::get_gain)
        .def("reset_to_home", &Arx5HighLevel::reset_to_home)
        .def("set_to_damping", &Arx5HighLevel::set_to_damping)
        .def("set_log_level", &Arx5HighLevel::set_log_level);
    py::class_<Arx5Solver>(m, "Arx5Solver")
        .def(py::init<>())
        .def("init_solver", &Arx5Solver::init_solver)
        .def("inverse_dynamics", &Arx5Solver::inverse_dynamics)
        .def("forward_kinematics", &Arx5Solver::forward_kinematics)
        .def("inverse_kinematics", &Arx5Solver::inverse_kinematics);
}