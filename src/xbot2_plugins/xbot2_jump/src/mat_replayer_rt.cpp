#include "mat_replayer_rt.h"

#include <math.h> 

void MatReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
    _pause_time = 0.0;
    _approach_traj_time = 0.0;
}

void MatReplayerRt::reset_flags()
{

    _approach_traj_started = false;
    _approach_traj_finished = false;

    _traj_started = false;
    _traj_finished = false;

    _pause_started = false;
    _pause_finished = false;

    _jump = false;

    _sample_index = 0; // resetting samples index, in case the plugin stopped and started again

    _approach_traj_time = 0;

}


void MatReplayerRt::update_clocks()
{
    // Update time(s)
    _loop_time += _plugin_dt;
    
    if(_pause_started && !_pause_finished)
    {
        _pause_time += _plugin_dt;
    }

    if(_approach_traj_started && !_approach_traj_finished)
    {
        _approach_traj_time += _plugin_dt;
    }

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

    if(_pause_time >= _traj_pause_time)
    {
        _pause_finished = true;
        _pause_time = _pause_time - _traj_pause_time;
    }

    if(_approach_traj_time >= _approach_traj_exec_time)
    {
        _approach_traj_time = _approach_traj_exec_time;
    }
    
}

void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    
    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive"); 
    _resample = getParamOrThrow<bool>("~resample"); 
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");
    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
    // _cntrl_mode =  getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");
    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness"); 
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");
    _looped_traj = getParamOrThrow<bool>("~looped_traj");
    _traj_pause_time = getParamOrThrow<double>("~traj_pause_time");
    _send_pos_ref = getParamOrThrow<bool>("~send_pos_ref");
    _send_vel_ref = getParamOrThrow<bool>("~send_vel_ref");
    _send_eff_ref = getParamOrThrow<bool>("~send_eff_ref");

}

void MatReplayerRt::update_state()
{
    // "sensing" the robot
    _robot->sense();
    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);
    
}

void MatReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/MatReplayerRt", opt); // date-time automatically appended
    }
    else
    {
        _dump_logger = MatLogger2::MakeLogger(_mat_path + _dump_mat_suffix, opt); // date-time automatically appended
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("stop_stiffness", _stop_stiffness);
    _dump_logger->add("stop_damping", _stop_damping);

    _dump_logger->create("plugin_time", 1);
    _dump_logger->create("replay_stiffness", _n_jnts_model);
    _dump_logger->create("replay_damping", _n_jnts_model);
    _dump_logger->create("q_p_meas", _n_jnts_model);
    _dump_logger->create("q_p_dot_meas", _n_jnts_model);
    _dump_logger->create("tau_meas", _n_jnts_model);
    _dump_logger->create("q_p_cmd", _n_jnts_model);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_model);
    _dump_logger->create("tau_cmd", _n_jnts_model);

    auto dscrptn_files_cell = XBot::matlogger2::MatData::make_cell(4);
    dscrptn_files_cell[0] = _mat_path;
    dscrptn_files_cell[1] = _mat_name;
    dscrptn_files_cell[2] = _robot->getUrdfPath();
    dscrptn_files_cell[3] = _robot->getSrdfPath();
    _dump_logger->save("description_files", dscrptn_files_cell);

}

void MatReplayerRt::add_data2dump_logger()
{
    
    _dump_logger->add("replay_stiffness", _replay_stiffness);
    _dump_logger->add("replay_damping", _replay_damping);
    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);
    _dump_logger->add("plugin_time", _loop_time);

    if (_traj_started && !_traj_finished)
    { // trajectory is being published
        
        if (_sample_index <= (_traj.get_n_nodes() - 1))
        { // commands have been computed

            if (_is_first_jnt_passive)
            { // remove first joint from logged commands

                _dump_logger->add("q_p_cmd", _q_p_cmd.tail(_n_jnts_model));
                _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd.tail(_n_jnts_model));
                _dump_logger->add("tau_cmd", _tau_cmd.tail(_n_jnts_model));

            }
            else
            {
                
                _dump_logger->add("q_p_cmd", _q_p_cmd.tail(_n_jnts_model));
                _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd.tail(_n_jnts_model));
                _dump_logger->add("tau_cmd", _tau_cmd.tail(_n_jnts_model));

            }

        }

    }

}

void MatReplayerRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    _jump_now_srv = _ros->advertiseService(
        "my_jump_now",
        &MatReplayerRt::on_jump_msg_rcvd,
        this,
        &_queue);

}

bool  MatReplayerRt::on_jump_msg_rcvd(const awesome_leg::JumpNowRequest& req,
                    awesome_leg::JumpNowResponse& res)
{

    _jump = req.jump_now;

    if (req.jump_now)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n Received jump signal! Hopefully the robot won't break...\n");

        res.message = "Starting replaying of jump trajectory!";

        _q_p_init_appr_traj = _q_p_meas; // initial position for the approach traj.
        _q_p_trgt_appr_traj = _q_p_ref.block(1, 0, _n_jnts_model, 1); // target pos. for the approach traj

        _approach_traj_started = true;
        
    }

    res.success = true;

    return true; 
}

void MatReplayerRt::load_opt_data()
{   

    _traj = plugin_utils::TrajLoader(_mat_path + _mat_name, true, 0.0001, false);

    int n_traj_jnts = _traj.get_n_jnts();

    if(n_traj_jnts != _n_jnts_model) 
    {
        jwarn("The loaded trajectory has {} joints, while the robot has {} .\n Make sure to somehow select the right components!!",
        n_traj_jnts, _n_jnts_model);
    }

    if (_resample)
    { // resample input data at the plugin frequency (for now it sucks)

        _traj.resample(_plugin_dt, _q_p_ref, _q_p_dot_ref, _tau_ref); // just brute for linear interpolation for now (for safety, better to always use the same plugin_dt as the loaded trajectory)

    }
    else
    { // load raw data without changes

        Eigen::MatrixXd dt_opt;

        _traj.get_loaded_traj(_q_p_ref, _q_p_dot_ref, _tau_ref, dt_opt);

        jwarn("The loaded trajectory was generated with a dt of {} s, while the rt plugin runs at {} .\n ",
        dt_opt(0), _plugin_dt);

    }


}

void MatReplayerRt::saturate_effort()
{
    int input_sign = 1; // defaults to positive sign 

    for(int i = 0; i < _n_jnts_model; i++)
    {
        if (abs(_tau_cmd[i]) >= abs(_effort_lims[i]))
        {
            input_sign = (signbit(_tau_cmd[i])) ? -1: 1; 

            _tau_cmd[i] = input_sign * (abs(_effort_lims[i]) - _delta_effort_lim);
        }
    }
}

// void MatReplayerRt::compute_approach_traj_offline()
// {

//     _approach_traj = plugin_utils::PeisekahTrans(_q_p_meas, _q_p_ref.block(1, 0, _n_jnts_model, 1), _approach_traj_exec_time, _plugin_dt); 

//     // _dump_logger->add("approach_traj", _approach_traj.get_traj());

// }

void MatReplayerRt::send_approach_trajectory()
{

    double phase = _approach_traj_time / _approach_traj_exec_time; // phase ([0, 1] inside the approach traj)

    if (_is_first_jnt_passive)
    { // send the last _n_jnts_model components

        _q_p_cmd = _peisekah_utils.compute_peisekah_vect_val(phase, _q_p_init_appr_traj, _q_p_trgt_appr_traj.tail(_n_jnts_model));
        
    }
    else{
        
        _q_p_cmd = _peisekah_utils.compute_peisekah_vect_val(phase, _q_p_init_appr_traj, _q_p_trgt_appr_traj);

    }

    _robot->setPositionReference(_q_p_cmd);
    
    _robot->move(); // Send commands to the robot
    
}

void MatReplayerRt::send_trajectory()
{

    if (_approach_traj_started && !_approach_traj_finished)
    { // still publishing the approach trajectory

        if (_approach_traj_time > _approach_traj_exec_time - 0.000001)
        {
            _approach_traj_finished = true; // finished approach traj
            _traj_started = true; // send actual trajectory
            _sample_index = 0; // resetting samples index
            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Approach trajectory finished...\n");
        }
        else
        {
            send_approach_trajectory();

        }
        
    }

    // Loop again thorugh the trajectory, if it is finished and the associated flag is active
    if (_traj_finished)
    { // finished publishing trajectory

        _pause_started = true;

        if (!_pause_finished)
        { // do nothing in this control loop

        }
        else 
        {

            reset_flags(); // reset flags

        }
        
    }

    // Publishing loaded traj samples
    if (_traj_started && !_traj_finished)
    { // publish current trajectory sample
        
        if (_sample_index > (_traj.get_n_nodes() - 1))
        { // reached the end of the trajectory
            
            _traj_finished = true;
            _sample_index = 0; // reset publish index (will be used to publish the loaded trajectory)
            
        }
        else
        { // send command
            
            // by default assign all commands anyway
            _q_p_cmd = _q_p_ref.col(_sample_index);
            _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index);
            _tau_cmd = _tau_ref.col(_sample_index);

            if (_is_first_jnt_passive)
            { // send the last _n_jnts_model components
                
                if (_send_eff_ref)
                {
                    _robot->setEffortReference(_tau_cmd.tail(_n_jnts_model));
                }

                if(_send_pos_ref)
                {  

                    _robot->setPositionReference(_q_p_cmd.tail(_n_jnts_model));
                }

                if(_send_vel_ref)
                {  

                    _robot->setVelocityReference(_q_p_dot_cmd.tail(_n_jnts_model));
                }

            }
            else{
                
                if (_send_eff_ref)
                {
                    _robot->setEffortReference(_tau_cmd);
                }

                if(_send_pos_ref)
                {  

                    _robot->setPositionReference(_q_p_cmd);
                }

                if(_send_vel_ref)
                {  

                    _robot->setVelocityReference(_q_p_dot_cmd);
                }

            }
            
            _robot->setStiffness(_replay_stiffness); // necessary at each loop (for some reason)
            _robot->setDamping(_replay_damping);

            saturate_effort(); // perform input torque saturation

            _robot->move(); // Send commands to the robot
        
        }

    }

}

bool MatReplayerRt::on_initialize()
{   

    _plugin_dt = getPeriodSec();

    _n_jnts_model = _robot->getJointNum();

    _robot->getEffortLimits(_effort_lims);

    init_nrt_ros_bridge();

    get_params_from_config(); // load params from yaml file
    
    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    _peisekah_utils = plugin_utils::PeisekahTrans();

    return true;
    
}

void MatReplayerRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    // setting the control mode to effort + velocity + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() + 
            ControlMode::Damping());
    _robot->setStiffness(_replay_stiffness);
    _robot->setDamping(_replay_damping);

    update_state(); // read current jnt positions and velocities
    
    // compute_approach_traj_offline(); // based on the current state, compute a smooth transition to the\\
    // first trajectory position sample

    // Move on to run()
    start_completed();
    
}

void MatReplayerRt::run()
{  

    update_state(); // read current jnt positions and velocities

    _queue.run();

    if (_jump) // only jump if a positive jump signal was received
    {
        send_trajectory();
    }
    
    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    _sample_index++; // incrementing loop counter

}

void MatReplayerRt::on_stop()
{
    // Read the current state
    _robot->sense();

    // Setting references before exiting
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
    _robot->setPositionReference(_q_p_meas);

    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void MatReplayerRt::stopping()
{

    stop_completed();
}

void MatReplayerRt::on_abort()
{
    // Read the current state
    _robot->sense();

    // Setting references before exiting
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
    _robot->setPositionReference(_q_p_meas);

    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void MatReplayerRt::on_close()
{
    jinfo("Closing MatReplayerRt");
}

XBOT2_REGISTER_PLUGIN(MatReplayerRt, mat_replayer_rt)

