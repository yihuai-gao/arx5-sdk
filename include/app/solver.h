#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include "utils.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Core>
#include <string.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <time.h>
#include <iostream>
#include <fstream>
#include <memory.h>
#include <dirent.h>
#include <vector>

enum arx5_state
{
    NORMAL,
    OUT_RANGE,
    OUT_BOUNDARY
}; // NORMAL：正常 OUT_RANGE：超出机械臂控制距离 OUT_BOUNDARY：关节锁死

#define FORWARD 0
#define DOWNWARD 1

#define filter_cmd 0.3f

class Arx5Solver
{

public:
    Arx5Solver(void) = default;
    ~Arx5Solver() = default;

    void init_solver(std::string urdf_file);
    Vec6d inverse_dynamics(Vec6d joint_pos, Vec6d joint_vel, Vec6d joint_acc);
    std::tuple<bool, Vec6d> inverse_kinematics(Vec6d target_pose_6d, Vec6d current_joint_pos);
    Vec6d forward_kinematics(Vec6d joint_pos);

private:
    // parameters for ik solver
    const double _EPS = 1E-5;
    const int _MAXITER = 500;
    const double _EPS_JOINTS = 1E-15;
    const double _MAX_TORQUE = 15.0f;
    const double _JOINT_POS_TOLERANCE = 0.01;

    KDL::Tree _tree;
    KDL::Chain _chain;
    KDL::Chain _chain_without_end_effector;

    std::shared_ptr<KDL::ChainIkSolverPos_LMA> _ik_solver;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> _fk_solver;
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> _fk_vel_solver;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> _ik_acc_solver;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> _jac_dot_solver;
    std::shared_ptr<KDL::ChainIdSolver_RNE> _id_solver;

    KDL::Frame _init_frame;
};

#endif