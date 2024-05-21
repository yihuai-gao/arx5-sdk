#include "app/joint_controller.h"
#include "utils.h"
#include <sys/syscall.h>
#include <sys/types.h>

Arx5JointController::Arx5JointController(std::string can_name)
    : _can_handle(can_name),
      _logger(spdlog::stdout_color_mt(std::string("ARX5_") + can_name))
{

    // Enable motor 5, 6, 7
    _can_handle.Enable_Moto(0x05);
    usleep(1000);
    _can_handle.Enable_Moto(0x06);
    usleep(1000);
    _can_handle.Enable_Moto(0x07);
    usleep(1000);
    _can_handle.Enable_Moto(0x08);
    usleep(1000);

    Gain gain = Gain();
    gain.kd = DEFAULT_KD;
    set_gain(gain); // set to damping by default
    _send_recv();
    _background_send_recv = std::thread(&Arx5JointController::_background_send_recv_task, this);
    _logger->set_pattern("[%H:%M:%S %n %^%l%$] %v");
    _logger->info("Background send_recv task is running at ID: {}", syscall(SYS_gettid));
}

Arx5JointController::~Arx5JointController()
{
    Gain damping_gain = Gain();
    damping_gain.kd = DEFAULT_KD;
    damping_gain.kd[1] *= 3;
    damping_gain.kd[2] *= 3;
    damping_gain.kd[3] *= 1.5;
    _logger->info("Set to damping before exit");
    set_gain(damping_gain);
    set_joint_cmd(JointState());
    _enable_gravity_compensation = false;
    sleep_ms(2000);
    _destroy_background_threads = true;
    _background_send_recv.join();
    _logger->info("background send_recv task joined");
}

JointState Arx5JointController::get_state()
{
    std::lock_guard<std::mutex> guard(_state_mutex);
    return _joint_state;
}

void Arx5JointController::send_recv_once()
{
    if (_background_send_recv_running)
    {
        std::cout << "send_recv task is already running in background. send_recv_once is ignored." << std::endl;
        return;
    }
    _send_recv();
    _check_current();
}

void Arx5JointController::_update_output_cmd()
{
    std::lock_guard<std::mutex> guard_cmd(_cmd_mutex);

    JointState prev_output_cmd = _output_joint_cmd;

    _output_joint_cmd = _input_joint_cmd;

    if (_enable_gravity_compensation && _solver != nullptr)
    {
        Vec6d gravity_torque = _solver->inverse_dynamics(_joint_state.pos, Vec6d::Zero(), Vec6d::Zero());
        _output_joint_cmd.torque += gravity_torque;
    }

    if (_enable_vel_clipping)
    {
        double dt = JOINT_CONTROLLER_DT;
        for (int i = 0; i < 6; ++i)
        {
            if (_gain.kp[i] > 0)
            {

                double delta_pos = _input_joint_cmd.pos[i] - prev_output_cmd.pos[i];
                double max_vel = JOINT_VEL_MAX[i];
                if (std::abs(delta_pos) > max_vel * dt)
                {
                    _output_joint_cmd.pos[i] = prev_output_cmd.pos[i] + max_vel * dt * delta_pos / std::abs(delta_pos);
                    _logger->debug("Joint {} pos {:.3f} pos cmd clipped: {:.3f} to {:.3f}", i, _joint_state.pos[i], _input_joint_cmd.pos[i], _output_joint_cmd.pos[i]);
                }
            }
            else
            {
                _output_joint_cmd.pos[i] = _joint_state.pos[i];
            }
        }
        if (_gain.gripper_kp > 0)
        {
            double gripper_delta_pos = _input_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos;
            if (std::abs(gripper_delta_pos) / dt > GRIPPER_VEL_MAX)
            {
                _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos + GRIPPER_VEL_MAX * dt * gripper_delta_pos / std::abs(gripper_delta_pos);
                _logger->debug("Gripper pos cmd clipped: {:.3f} to {:.3f}", _input_joint_cmd.gripper_pos, _output_joint_cmd.gripper_pos);
            }
        }
        else
        {
            _output_joint_cmd.gripper_pos = _joint_state.gripper_pos;
        }
    }

    // Joint pos clipping
    for (int i = 0; i < 6; ++i)
    {
        if (_output_joint_cmd.pos[i] < JOINT_POS_MIN[i])
        {
            _logger->debug("Joint {} pos {:.3f} pos cmd clipped from {:.3f} to min {:.3f}", i, _joint_state.pos[i], _output_joint_cmd.pos[i], JOINT_POS_MIN[i]);
            _output_joint_cmd.pos[i] = JOINT_POS_MIN[i];
        }
        else if (_output_joint_cmd.pos[i] > JOINT_POS_MAX[i])
        {
            _logger->debug("Joint {} pos {:.3f} pos cmd clipped from {:.3f} to max {:.3f}", i, _joint_state.pos[i], _output_joint_cmd.pos[i], JOINT_POS_MAX[i]);
            _output_joint_cmd.pos[i] = JOINT_POS_MAX[i];
        }
    }
    // Gripper pos clipping
    if (_output_joint_cmd.gripper_pos < 0)
    {
        _logger->debug("Gripper pos cmd clipped from {:.3f} to min: {:.3f}", _output_joint_cmd.gripper_pos, 0.0);
        _output_joint_cmd.gripper_pos = 0;
    }
    else if (_output_joint_cmd.gripper_pos > GRIPPER_WIDTH)
    {
        _logger->debug("Gripper pos cmd clipped from {:.3f} to max: {:.3f}", _output_joint_cmd.gripper_pos, GRIPPER_WIDTH);
        _output_joint_cmd.gripper_pos = GRIPPER_WIDTH;
    }
    if (std::abs(_joint_state.gripper_torque) > GRIPPER_TORQUE_MAX / 2)
    {
        double sign = _joint_state.gripper_torque > 0 ? 1 : -1;                         // -1 for closing blocked, 1 for opening blocked
        double delta_pos = _output_joint_cmd.gripper_pos - prev_output_cmd.gripper_pos; // negative for closing, positive for opening
        if (delta_pos * sign > 0)
        {
            _logger->debug("Gripper torque is too large, gripper pos cmd is not updated");
            _output_joint_cmd.gripper_pos = prev_output_cmd.gripper_pos;
        }
    }

    if (_enable_torque_clipping)
    {
        // Torque clipping
        for (int i = 0; i < 6; ++i)
        {
            if (_output_joint_cmd.torque[i] > JOINT_TORQUE_MAX[i])
            {
                _logger->debug("Joint {} torque cmd clipped from {:.3f} to max {:.3f}", i, _output_joint_cmd.torque[i], JOINT_TORQUE_MAX[i]);
                _output_joint_cmd.torque[i] = JOINT_TORQUE_MAX[i];
            }
            else if (_output_joint_cmd.torque[i] < -JOINT_TORQUE_MAX[i])
            {
                _logger->debug("Joint {} torque cmd clipped from {:.3f} to min {:.3f}", i, _output_joint_cmd.torque[i], -JOINT_TORQUE_MAX[i]);
                _output_joint_cmd.torque[i] = -JOINT_TORQUE_MAX[i];
            }
        }
    }
}

double Arx5JointController::get_timestamp()
{
    return double(get_time_us() - _start_time_us) / 1e6;
}

void Arx5JointController::_send_recv()
{
    // TODO: in the motor documentation, there shouldn't be these torque constant. Torque will go directly into the motors
    const double torque_constant1 = 1.4;   // Nm/A, only for the bottom 3 motors
    const double torque_constant2 = 0.424; // Nm/A, only for the top 3 motors

    int start_time_us = get_time_us();
    _update_output_cmd();
    int update_cmd_time_us = get_time_us();
    int communicate_sleep_us = 150;

    int start_send_motor_0_time_us = get_time_us();
    _can_handle.Send_moto_Cmd1(_MOTOR_ID[0], _gain.kp[0], _gain.kd[0], _output_joint_cmd.pos[0], _output_joint_cmd.vel[0], _output_joint_cmd.torque[0] / torque_constant1);
    int send_motor_0_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_0_time_us - start_send_motor_0_time_us));

    int start_send_motor_1_time_us = get_time_us();
    _can_handle.Send_moto_Cmd1(_MOTOR_ID[1], _gain.kp[1], _gain.kd[1], _output_joint_cmd.pos[1], _output_joint_cmd.vel[1], _output_joint_cmd.torque[1] / torque_constant1);
    int send_motor_1_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_1_time_us - start_send_motor_1_time_us));

    int start_send_motor_2_time_us = get_time_us();
    _can_handle.Send_moto_Cmd1(_MOTOR_ID[2], _gain.kp[2], _gain.kd[2], _output_joint_cmd.pos[2], _output_joint_cmd.vel[2], _output_joint_cmd.torque[2] / torque_constant1);
    int send_motor_2_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_2_time_us - start_send_motor_2_time_us));

    int start_send_motor_3_time_us = get_time_us();
    _can_handle.Send_moto_Cmd2(_MOTOR_ID[3], _gain.kp[3], _gain.kd[3], _output_joint_cmd.pos[3], _output_joint_cmd.vel[3], _output_joint_cmd.torque[3] / torque_constant2);
    int send_motor_3_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_3_time_us - start_send_motor_3_time_us));

    int start_send_motor_4_time_us = get_time_us();
    _can_handle.Send_moto_Cmd2(_MOTOR_ID[4], _gain.kp[4], _gain.kd[4], _output_joint_cmd.pos[4], _output_joint_cmd.vel[4], _output_joint_cmd.torque[4] / torque_constant2);
    int send_motor_4_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_4_time_us - start_send_motor_4_time_us));

    int start_send_motor_5_time_us = get_time_us();
    _can_handle.Send_moto_Cmd2(_MOTOR_ID[5], _gain.kp[5], _gain.kd[5], _output_joint_cmd.pos[5], _output_joint_cmd.vel[5], _output_joint_cmd.torque[5] / torque_constant2);
    int send_motor_5_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_5_time_us - start_send_motor_5_time_us));

    int start_send_motor_6_time_us = get_time_us();
    double gripper_motor_pos = _output_joint_cmd.gripper_pos / GRIPPER_WIDTH * _GRIPPER_OPEN_READOUT;
    _can_handle.Send_moto_Cmd2(_MOTOR_ID[6], _gain.gripper_kp, _gain.gripper_kd, gripper_motor_pos, 0, 0);
    int send_motor_6_time_us = get_time_us();

    sleep_us(communicate_sleep_us - (send_motor_6_time_us - start_send_motor_6_time_us));

    int start_get_motor_msg_time_us = get_time_us();
    std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
    int get_motor_msg_time_us = get_time_us();

    // _logger->trace("update_cmd: {} us, send_motor_0: {} us, send_motor_1: {} us, send_motor_2: {} us, send_motor_3: {} us, send_motor_4: {} us, send_motor_5: {} us, send_motor_6: {} us, get_motor_msg: {} us",
    //                update_cmd_time_us - start_time_us, send_motor_0_time_us - start_send_motor_0_time_us, send_motor_1_time_us - start_send_motor_1_time_us, send_motor_2_time_us - start_send_motor_2_time_us,
    //                send_motor_3_time_us - start_send_motor_3_time_us, send_motor_4_time_us - start_send_motor_4_time_us, send_motor_5_time_us - start_send_motor_5_time_us, send_motor_6_time_us - start_send_motor_6_time_us, get_motor_msg_time_us - start_get_motor_msg_time_us);

    std::lock_guard<std::mutex> guard_state(_state_mutex);

    _joint_state.pos[0] = motor_msg[0].angle_actual_rad;
    _joint_state.pos[1] = motor_msg[1].angle_actual_rad;
    _joint_state.pos[2] = motor_msg[3].angle_actual_rad;
    _joint_state.pos[3] = motor_msg[4].angle_actual_rad;
    _joint_state.pos[4] = motor_msg[5].angle_actual_rad;
    _joint_state.pos[5] = motor_msg[6].angle_actual_rad;
    _joint_state.gripper_pos = motor_msg[7].angle_actual_rad / _GRIPPER_OPEN_READOUT * GRIPPER_WIDTH;

    _joint_state.vel[0] = motor_msg[0].speed_actual_rad;
    _joint_state.vel[1] = motor_msg[1].speed_actual_rad;
    _joint_state.vel[2] = motor_msg[3].speed_actual_rad;
    _joint_state.vel[3] = motor_msg[4].speed_actual_rad;
    _joint_state.vel[4] = motor_msg[5].speed_actual_rad;
    _joint_state.vel[5] = motor_msg[6].speed_actual_rad;
    _joint_state.gripper_vel = motor_msg[7].speed_actual_rad / _GRIPPER_OPEN_READOUT * GRIPPER_WIDTH;

    // HACK: just to match the values (there must be something wrong)
    _joint_state.torque[0] = motor_msg[0].current_actual_float * torque_constant1 * torque_constant1;
    _joint_state.torque[1] = motor_msg[1].current_actual_float * torque_constant1 * torque_constant1;
    _joint_state.torque[2] = motor_msg[3].current_actual_float * torque_constant1 * torque_constant1;
    _joint_state.torque[3] = motor_msg[4].current_actual_float * torque_constant2;
    _joint_state.torque[4] = motor_msg[5].current_actual_float * torque_constant2;
    _joint_state.torque[5] = motor_msg[6].current_actual_float * torque_constant2;
    _joint_state.gripper_torque = motor_msg[7].current_actual_float * torque_constant2;
    _joint_state.timestamp = get_timestamp();
}

void Arx5JointController::_check_current()
{
    bool over_current = false;
    for (int i = 0; i < 6; ++i)
    {
        if (std::abs(_joint_state.torque[i]) > JOINT_TORQUE_MAX[i])
        {
            over_current = true;
            _logger->error("Over current detected once on joint {}, current: {:.3f}", i, _joint_state.torque[i]);
            break;
        }
    }
    if (std::abs(_joint_state.gripper_torque) > GRIPPER_TORQUE_MAX)
    {
        over_current = true;
        _logger->error("Over current detected once on gripper, current: {:.3f}", _joint_state.gripper_torque);
    }
    if (over_current)
    {
        _over_current_cnt++;
        if (_over_current_cnt > OVER_CURRENT_CNT_MAX)
        {
            _logger->error("Over current detected, robot is set to damping. Please restart the program.");
            Gain damping_gain;
            damping_gain.kd = DEFAULT_KD;
            damping_gain.kd[1] *= 3;
            damping_gain.kd[2] *= 3;
            damping_gain.kd[3] *= 1.5;
            set_gain(damping_gain);

            while (true)
            {
                _send_recv();
                sleep_ms(5);
            }
        }
    }
    else
    {
        _over_current_cnt = 0;
    }
}

void Arx5JointController::_background_send_recv_task()
{
    while (!_destroy_background_threads)
    {
        int start_time_us = get_time_us();
        if (_background_send_recv_running)
        {
            _send_recv();
            _check_current();
        }
        int elapsed_time_us = get_time_us() - start_time_us;
        int sleep_time_us = int(JOINT_CONTROLLER_DT * 1e6) - elapsed_time_us;
        if (sleep_time_us > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us));
        }
        else
        {
            _logger->debug("Background send_recv task is running too slow, time: {} us", elapsed_time_us);
        }
    }
}

void Arx5JointController::enable_background_send_recv()
{
    _logger->info("Enable background send_recv");
    _background_send_recv_running = true;
}

void Arx5JointController::disable_background_send_recv()
{
    _logger->info("Disable background send_recv");
    _background_send_recv_running = false;
}

void Arx5JointController::set_joint_cmd(JointState new_cmd)
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    if (new_cmd.gripper_vel != 0 || new_cmd.gripper_torque != 0)
    {
        _logger->warn("Gripper vel and torque control is not supported yet.");
        new_cmd.gripper_vel = 0;
        new_cmd.gripper_torque = 0;
    }
    _input_joint_cmd = new_cmd;
}

std::tuple<JointState, JointState> Arx5JointController::get_joint_cmd()
{
    std::lock_guard<std::mutex> guard(_cmd_mutex);
    return std::make_tuple(_input_joint_cmd, _output_joint_cmd);
}

Gain Arx5JointController::get_gain()
{
    return _gain;
}

void Arx5JointController::set_gain(Gain new_gain)
{
    // std::cout << "Set new gain: kp: " << new_gain.kp.transpose() << ", kd: " << new_gain.kd.transpose() << std::endl;
    _gain = new_gain;
}

Vec6d Arx5JointController::clip_joint_pos(Vec6d pos)
{
    Vec6d pos_clip = pos.cwiseMax(JOINT_POS_MIN).cwiseMin(JOINT_POS_MAX);
    return pos;
}

void Arx5JointController::reset_to_home()
{
    JointState cmd;
    Gain gain;
    JointState init_state = get_state();
    Gain init_gain = get_gain();
    double init_gripper_kp = _gain.gripper_kp;
    double init_gripper_kd = _gain.gripper_kd;
    Gain target_gain = Gain(DEFAULT_KP, DEFAULT_KD, DEFAULT_GRIPPER_KP, DEFAULT_GRIPPER_KD);
    JointState target_state;

    // calculate the maximum joint position error
    double max_pos_error = (init_state.pos - Vec6d::Zero()).cwiseAbs().maxCoeff();
    max_pos_error = std::max(max_pos_error, init_state.gripper_pos * 2 / GRIPPER_WIDTH);
    // interpolate from current kp kd to default kp kd in max(max_pos_error, 0.5)s
    // and keep the target for max(max_pos_error, 0.5)s
    double step_num = std::max(max_pos_error, 0.5) / JOINT_CONTROLLER_DT;
    _logger->info("Start reset to home in {:.3f}s, max_pos_error: {:.3f}", std::max(max_pos_error, double(0.5)) + 0.5, max_pos_error);

    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = true;
    for (int i = 0; i <= step_num; ++i)
    {
        double alpha = double(i) / step_num;
        gain = init_gain * (1 - alpha) + target_gain * alpha;
        cmd = init_state * (1 - alpha) + target_state * alpha;
        set_gain(gain);
        set_joint_cmd(cmd);
        sleep_ms(5);
    }
    // Hardcode 0.5 s
    sleep_ms(500);
    _logger->info("Finish reset to home");
    _background_send_recv_running = prev_running;
}

void Arx5JointController::set_to_damping()
{
    JointState cmd;
    JointState state;
    Gain gain;
    JointState init_state = get_state();
    Gain init_gain = get_gain();
    Gain target_gain;
    target_gain.kd = DEFAULT_KD;
    _logger->info("Start set to damping");
    //  interpolate from current kp kd to default kp kd in 0.5s
    bool prev_running = _background_send_recv_running;
    int step_num = 20; // 0.1s in total
    for (int i = 0; i <= step_num; ++i)
    {
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

bool Arx5JointController::is_damping()
{
    return _gain.kp.isZero() && _gain.gripper_kp == 0 && _gain.gripper_kd == 0 &&
           _input_joint_cmd.torque.isZero() && _input_joint_cmd.vel.isZero() &&
           _output_joint_cmd.torque.isZero() && _output_joint_cmd.vel.isZero();
}

void Arx5JointController::calibrate_gripper()
{
    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = false;
    usleep(1000);
    for (int i = 0; i < 10; ++i)
    {
        _can_handle.Send_moto_Cmd2(8, 0, 0, 0, 0, 0);
        usleep(400);
    }
    _logger->info("Start calibrating gripper. Please fully close the gripper and press enter to continue");
    std::cin.get();
    _can_handle.Set_Zero(0x08);
    usleep(400);
    for (int i = 0; i < 10; ++i)
    {
        _can_handle.Send_moto_Cmd2(8, 0, 0, 0, 0, 0);
        usleep(400);
    }
    usleep(400);
    _logger->info("Finish setting zero point. Please fully open the gripper and press enter to continue");
    std::cin.get();

    for (int i = 0; i < 10; ++i)
    {
        _can_handle.Send_moto_Cmd2(8, 0, 0, 0, 0, 0);
        usleep(400);
    }
    std::array<OD_Motor_Msg, 10> motor_msg = _can_handle.get_motor_msg();
    std::cout << "Fully-open joint position readout: " << motor_msg[7].angle_actual_rad << std::endl;
    std::cout << "  Please update the _GRIPPER_OPEN_READOUT value in joint_controller.h to finish gripper calibration." << std::endl;
    if (prev_running)
    {
        _background_send_recv_running = true;
    }
}

void Arx5JointController::calibrate_joint(int joint_id)
{
    bool prev_running = _background_send_recv_running;
    _background_send_recv_running = false;
    usleep(1000);
    int motor_id = _MOTOR_ID[joint_id];
    for (int i = 0; i < 10; ++i)
    {
        if (joint_id < 3)
            _can_handle.Send_moto_Cmd1(motor_id, 0, 0, 0, 0, 0);
        else
            _can_handle.Send_moto_Cmd2(motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    _logger->info("Start calibrating joint {}. Please move the joint to the home position and press enter to continue", joint_id);
    std::cin.get();
    if (joint_id < 3)
        _can_handle.CAN_cmd_init(motor_id, 0x03);
    else
        _can_handle.Set_Zero(motor_id);
    usleep(400);
    for (int i = 0; i < 10; ++i)
    {
        if (joint_id < 3)
            _can_handle.Send_moto_Cmd1(motor_id, 0, 0, 0, 0, 0);
        else
            _can_handle.Send_moto_Cmd2(motor_id, 0, 0, 0, 0, 0);
        usleep(400);
    }
    usleep(400);
    _logger->info("Finish setting zero point for joint {}.", joint_id);
    if (prev_running)
    {
        _background_send_recv_running = true;
    }
}

void Arx5JointController::set_log_level(spdlog::level::level_enum log_level)
{
    _logger->set_level(log_level);
}

void Arx5JointController::enable_gravity_compensation(std::string urdf_path)
{
    _logger->info("Enable gravity compensation");
    _logger->info("Loading urdf from {}", urdf_path);
    _solver = std::make_shared<Arx5Solver>(urdf_path);
    _enable_gravity_compensation = true;
}

void Arx5JointController::disable_gravity_compensation()
{
    _logger->info("Disable gravity compensation");
    _enable_gravity_compensation = false;
    _solver.reset();
}