#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "app/high_level.h"
#include "app/joint_controller.h"
#include "spdlog/spdlog.h"
#include "utils.h"
namespace py = pybind11;
using namespace arx;
using Vec6d = Eigen::Matrix<double, 6, 1>;
PYBIND11_MODULE(arx5_interface, m) {
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
  m.attr("JOINT_CONTROLLER_DT") = JOINT_CONTROLLER_DT;
  py::enum_<spdlog::level::level_enum>(m, "LogLevel")
      .value("TRACE", spdlog::level::level_enum::trace)
      .value("DEBUG", spdlog::level::level_enum::debug)
      .value("INFO", spdlog::level::level_enum::info)
      .value("WARNING", spdlog::level::level_enum::warn)
      .value("ERROR", spdlog::level::level_enum::err)
      .value("CRITICAL", spdlog::level::level_enum::critical)
      .value("OFF", spdlog::level::level_enum::off)
      .export_values();
  py::class_<JointState>(m, "JointState")
      .def(py::init<>())
      .def(py::init<Vec6d, Vec6d, Vec6d, double>())
      .def_readwrite("timestamp", &JointState::timestamp)
      .def_readwrite("gripper_pos", &JointState::gripper_pos)
      .def_readwrite("gripper_vel", &JointState::gripper_vel)
      .def_readwrite("gripper_torque", &JointState::gripper_torque)
      .def("__add__", [](const JointState& self,
                         const JointState& other) { return self + other; })
      .def("__mul__", [](const JointState& self,
                         const float& scalar) { return self * scalar; })
      .def("pos", &JointState::get_pos_ref, py::return_value_policy::reference)
      .def("vel", &JointState::get_vel_ref, py::return_value_policy::reference)
      .def("torque", &JointState::get_torque_ref,
           py::return_value_policy::reference);
  py::class_<Gain>(m, "Gain")
      .def(py::init<>())
      .def(py::init<Vec6d, Vec6d, double, double>())
      .def_readwrite("gripper_kp", &Gain::gripper_kp)
      .def_readwrite("gripper_kd", &Gain::gripper_kd)
      .def("__add__",
           [](const Gain& self, const Gain& other) { return self + other; })
      .def("__mul__",
           [](const Gain& self, const float& scalar) { return self * scalar; })
      .def("kp", &Gain::get_kp_ref, py::return_value_policy::reference)
      .def("kd", &Gain::get_kd_ref, py::return_value_policy::reference);
  py::class_<Arx5JointController>(m, "Arx5JointController")
      .def(py::init<const std::string&, const std::string&>())
      .def("send_recv_once", &Arx5JointController::send_recv_once)
      .def("enable_background_send_recv",
           &Arx5JointController::enable_background_send_recv)
      .def("disable_background_send_recv",
           &Arx5JointController::disable_background_send_recv)
      .def("enable_gravity_compensation",
           &Arx5JointController::enable_gravity_compensation)
      .def("disable_gravity_compensation",
           &Arx5JointController::disable_gravity_compensation)
      .def("get_state", &Arx5JointController::get_state)
      .def("get_timestamp", &Arx5JointController::get_timestamp)
      .def("set_joint_cmd", &Arx5JointController::set_joint_cmd)
      .def("get_joint_cmd", &Arx5JointController::get_joint_cmd)
      .def("set_gain", &Arx5JointController::set_gain)
      .def("get_gain", &Arx5JointController::get_gain)
      .def("clip_joint_pos", &Arx5JointController::clip_joint_pos)
      .def("reset_to_home", &Arx5JointController::reset_to_home)
      .def("set_to_damping", &Arx5JointController::set_to_damping)
      .def("set_log_level", &Arx5JointController::set_log_level)
      .def("calibrate_joint", &Arx5JointController::calibrate_joint)
      .def("calibrate_gripper", &Arx5JointController::calibrate_gripper);
  py::class_<HighState>(m, "HighState")
      .def(py::init<>())
      .def(py::init<Vec6d, double>())
      .def_readwrite("timestamp", &HighState::timestamp)
      .def_readwrite("gripper_pos", &HighState::gripper_pos)
      .def_readwrite("gripper_vel", &HighState::gripper_vel)
      .def_readwrite("gripper_torque", &HighState::gripper_torque)
      .def("__add__", [](const HighState& self,
                         const HighState& other) { return self + other; })
      .def("__mul__", [](const HighState& self,
                         const float& scalar) { return self * scalar; })
      .def("pose_6d", &HighState::get_pose_6d_ref,
           py::return_value_policy::reference);
  py::class_<Arx5HighLevel>(m, "Arx5HighLevel")
      .def(py::init<const std::string&, const std::string&,
                    const std::string&>())
      .def("set_high_cmd", &Arx5HighLevel::set_high_cmd)
      .def("get_high_state", &Arx5HighLevel::get_high_state)
      .def("get_joint_state", &Arx5HighLevel::get_joint_state)
      .def("get_high_cmd", &Arx5HighLevel::get_high_cmd)
      .def("get_joint_cmd", &Arx5HighLevel::get_joint_cmd)
      .def("get_timestamp", &Arx5HighLevel::get_timestamp)
      .def("set_gain", &Arx5HighLevel::set_gain)
      .def("get_gain", &Arx5HighLevel::get_gain)
      .def("reset_to_home", &Arx5HighLevel::reset_to_home)
      .def("set_to_damping", &Arx5HighLevel::set_to_damping);
  py::class_<Arx5Solver>(m, "Arx5Solver")
      .def(py::init<const std::string&>())
      .def("inverse_dynamics", &Arx5Solver::inverse_dynamics)
      .def("forward_kinematics", &Arx5Solver::forward_kinematics)
      .def("inverse_kinematics", &Arx5Solver::inverse_kinematics);
}