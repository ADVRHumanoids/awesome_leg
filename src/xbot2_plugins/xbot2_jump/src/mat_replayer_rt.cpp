#include "mat_replayer_rt.h"

#include <math.h> 

void MatReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
    _pause_time = 0.0;
}

void MatReplayerRt::reset_flags()
{

    _first_run = true; // reset flag in case the plugin is run multiple times
    _approach_traj_started = false;
    _approach_traj_finished = false;
    _traj_started = false;
    _traj_finished = false;
    _pause_started = false;
    _pause_finished = false;
    _send_eff_ref = false;
    _jump = false;
    _recompute_approach_traj = true;

    _sample_index = 0; // resetting samples index, in case the plugin stopped and started again

}

void MatReplayerRt::update_clocks()
{
    // Update time(s)
    _loop_time += _plugin_dt;
    
    if(_pause_started && !_pause_finished)
    {
        _pause_time += _plugin_dt;
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
    
}

void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive"); 
    _resample = getParamOrThrow<bool>("~resample"); 
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");
    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
    _approach_traj_target = getParamOrThrow<Eigen::VectorXd>("~approach_traj_target");
    // _cntrl_mode =  getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");
    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness"); 
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");
    _looped_traj = getParamOrThrow<bool>("~looped_traj");
    _traj_pause_time = getParamOrThrow<double>("~traj_pause_time");
    _send_eff_ref = getParamOrThrow<bool>("~send_eff_ref");

}

void MatReplayerRt::update_state()
{
    // "sensing" the robot
    _robot->sense();
    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);    
    
}

void MatReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _dump_logger = MatLogger2::MakeLogger("/tmp/MatReplayerRt", opt); // date-time automatically appended
    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->create("q_p_meas", _n_jnts_model);
    _dump_logger->create("q_p_dot_meas", _n_jnts_model);
    _dump_logger->add("q_ref", _q_p_ref); 
    _dump_logger->add("q_dot_ref", _q_p_dot_ref);
    _dump_logger->add("tau_ref", _tau_ref);
    // auto dscrptn_files_cell = XBot::matlogger2::MatData::make_cell(2);
    // dscrptn_files_cell[0] = _robot->getUrdfPath();
    // dscrptn_files_cell[1] = _robot->getSrdfPath();
    // _dump_logger->save("description_files", dscrptn_files_cell);

}

void MatReplayerRt::add_data2dump_logger()
{
    
    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("stop_stiffness", _stop_stiffness);
    _dump_logger->add("stop_damping", _stop_damping);
    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_meas);
    _dump_logger->add("plugin_time", _loop_time);

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
        
    }

    if (!req.jump_now)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n Stopping trajectory replay ...\n");

        res.message = "Stopping trajectory replay!";

        _recompute_approach_traj = true; // resetting flag so that a new approaching traj can be computed

        reset_flags();
    }

    res.success = true;

    return true; 
}

void MatReplayerRt::load_opt_data()
{   

    _traj = plugin_utils::TrajLoader(_mat_path + _mat_name);

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

void MatReplayerRt::saturate_input()
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

void MatReplayerRt::compute_approach_traj()
{

    _approach_traj = plugin_utils::PeisekahTrans(_q_p_meas, _approach_traj_target, _approach_traj_exec_time, _plugin_dt); 

    _dump_logger->add("approach_traj", _approach_traj.get_traj());

    _recompute_approach_traj = false;

}

void MatReplayerRt::send_approach_trajectory()
{

    if (_sample_index > (_approach_traj.get_n_nodes() - 1))
    { // reached the end of the trajectory

        _approach_traj_finished = true;
        _traj_started = true; // start to publish the loaded trajectory starting from the next control loop
        _sample_index = 0; // reset publish index (will be used to publish the loaded trajectory)

    }
    else
    {
        _q_p_cmd = _approach_traj.eval_at(_sample_index);

        if (_is_first_jnt_passive)
        { // send the last _n_jnts_model components
            _robot->setPositionReference(_q_p_cmd.tail(_n_jnts_model));
        }
        else{
            
            _robot->setPositionReference(_q_p_cmd);

        }
        

        _robot->move(); // Send commands to the robot
    }
    
}

void MatReplayerRt::send_trajectory()
{
    // first, set the control mode and stiffness when entering the first control loop (to be used during the approach traj)
    if (_first_run)
    { // first time entering the control loop
        
        _sample_index = 0;
        _approach_traj_started = true; // flag signaling the start of the approach trajectory
       
    }

    // Loop again thorugh the trajectory, if it is finished and the associated flag is active
    if (_traj_finished && _looped_traj)
    { // finished publishing trajectory

        _pause_started = true;

        if (!_pause_finished)
        { // do nothing in this control loop

        }
        else 
        {
            _sample_index = 0; // reset publishing index
            _traj_finished = false; // reset flag
            
            _pause_started = false;
            _pause_finished = false;
        }
        
    }

    // When the plugin is stopped from service, recompute a transition trajectory
    // from the current state
    if (_recompute_approach_traj)
    {
        
        compute_approach_traj(); // necessary if traj replay is stopped and started again from service (probably breaks rt performance)
    }

    if (_approach_traj_started && !_approach_traj_finished)
    { // still publishing the approach trajectory

        send_approach_trajectory();
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

            _q_p_cmd = _q_p_ref.col(_sample_index);

            // _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index);
            
            _tau_cmd = _tau_ref.col(_sample_index);

            if (_is_first_jnt_passive)
            { // send the last _n_jnts_model components
                _robot->setPositionReference(_q_p_cmd.tail(_n_jnts_model));

                if(_send_eff_ref)
                {
                    _robot->setEffortReference(_tau_cmd.tail(_n_jnts_model));
                }

            }
            else{
            
                _robot->setPositionReference(_q_p_cmd);

                if(_send_eff_ref)
                {
                    _robot->setEffortReference(_tau_cmd);
                }

            }
            
            _robot->setStiffness(_replay_stiffness); // necessary at each loop (for some reason)
            _robot->setDamping(_replay_damping);

            saturate_input();  

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

    return true;
}

void MatReplayerRt::starting()
{
    get_params_from_config(); // load params from yaml file
    
    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    init_dump_logger();

    reset_flags();

    init_clocks(); // initialize clocks timers

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort() + ControlMode::Stiffness() + 
            ControlMode::Damping());
    _robot->setStiffness(_replay_stiffness);
    _robot->setDamping(_replay_damping);

    update_state(); // read current jnt positions and velocities
    compute_approach_traj(); // based on the current state, compute a smooth transition to the\\
    first trajectory position sample

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

    if (_first_run == true & _jump)
    { // this is the end of the first control loop
        _first_run = false;
    }

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
    _first_run = true; 

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
}

void MatReplayerRt::on_close()
{
    jinfo("Closing MatReplayerRt");
}

XBOT2_REGISTER_PLUGIN(MatReplayerRt, mat_replayer_rt)