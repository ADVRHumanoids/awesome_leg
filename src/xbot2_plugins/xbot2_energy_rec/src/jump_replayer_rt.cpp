#include "jump_replayer_rt.h"

#include <math.h> 

void JumpReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _pause_time = 0.0;
    _approach_traj_time = 0.0;
    _smooth_imp_time = 0.0;
}

void JumpReplayerRt::init_vars()
{
    _q_p_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_safe_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_trgt_appr_traj = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_init_appr_traj =  Eigen::VectorXd::Zero(_n_jnts_robot);

    _meas_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);

    _ramp_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_strt_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_strt_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _meas_driver_temp = Eigen::VectorXd::Zero(_n_jnts_robot);

    _auxiliary_vector = Eigen::VectorXd::Zero(_n_jnts_robot);

}

void JumpReplayerRt::reset_flags()
{

    _approach_traj_started = false;
    _approach_traj_finished = false;

    _imp_traj_started = false;
    _imp_traj_finished = false;

    _traj_started = false;
    _traj_finished = false;

    _pause_started = false;
    _pause_finished = false;

    _jump = false;

    _sample_index = 0.0; // resetting samples index, in case the plugin stopped and started again

    _approach_traj_time = 0.0;

    _smooth_imp_time = 0.0;
    
    _jump_now = false;
    
    _is_first_trigger = false;

    _is_first_imp_ramp_loop = true;

}

void JumpReplayerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;
    
    if(_pause_started && !_pause_finished)
    {
        _pause_time += _plugin_dt;
    }

    if(_imp_traj_started && !_imp_traj_finished)
    {
        _smooth_imp_time += _plugin_dt;
    }

    if(_approach_traj_started && !_approach_traj_finished && _jump)
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

    if(_approach_traj_finished)
    {
        _approach_traj_time = _approach_traj_exec_time;
    }

    if(_imp_traj_finished)
    {
        _smooth_imp_time = _imp_ramp_time;
    }
    
}

void JumpReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 
    _srdf_path = getParamOrThrow<std::string>("~srdf_path"); 

    _urdf_path_ft_est = getParamOrThrow<std::string>("~urdf_path_ft_est");
    _srdf_path_ft_est = getParamOrThrow<std::string>("~srdf_path_ft_est");

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive"); 
    _resample = getParamOrThrow<bool>("~resample"); 
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");
    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
    // _cntrl_mode =  getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");

    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness"); 
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");
    _touchdown_stiffness = getParamOrThrow<Eigen::VectorXd>("~touchdown_stiffness"); 
    _touchdown_damping = getParamOrThrow<Eigen::VectorXd>("~touchdown_damping"); 

    _traj_pause_time = getParamOrThrow<double>("~traj_pause_time");
    _send_pos_ref = getParamOrThrow<bool>("~send_pos_ref");
    _send_vel_ref = getParamOrThrow<bool>("~send_vel_ref");
    _send_eff_ref = getParamOrThrow<bool>("~send_eff_ref");

    _tip_link_name = getParamOrThrow<std::string>("~tip_link_name"); 
    _base_link_name = getParamOrThrow<std::string>("~base_link_name");

    _imp_ramp_time = getParamOrThrow<double>("~imp_ramp_time");

    _reduce_dumped_sol_size = getParamOrThrow<bool>("~reduce_dumped_sol_size");

    _send_whole_traj = getParamOrThrow<bool>("~send_whole_traj");

    _verbose = getParamOrThrow<bool>("~verbose");

    _driver_temp_threshold = getParamOrThrow<Eigen::VectorXd>("~driver_temp_threshold");

}

void JumpReplayerRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t sim_found = _hw_type.find(sim_string);

    if (sim_found != std::string::npos) { // we are running the plugin in simulation

        _is_sim = true;
    }
    else // we are running on the real robot
    {
        _is_sim = false;
    }

}

void JumpReplayerRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "mr");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "jump_replayer_rt");
}

void JumpReplayerRt::update_state()
{    
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    _robot->getStiffness(_meas_stiffness); // used by the smooth imp. transitioner
    _robot->getDamping(_meas_damping);

    _robot->getTemperature(_meas_driver_temp); // getting driver temperature

}

void JumpReplayerRt::send_cmds()
{
    // always set impedance to the last setpoint to avoid issues
    _robot->setStiffness(_stiffness_setpoint); 
    _robot->setDamping(_damping_setpoint);

    if (_is_first_jnt_passive)
    { // send the last _n_jnts_robot components
        
        if (_send_eff_ref)
        {
            _auxiliary_vector = _tau_cmd.tail(_n_jnts_robot);
            _robot->setEffortReference(_auxiliary_vector);
        }

        if(_send_pos_ref)
        {  
            _auxiliary_vector = _q_p_cmd.tail(_n_jnts_robot);
            _robot->setPositionReference(_auxiliary_vector);
        }

        if(_send_vel_ref)
        {  
            _auxiliary_vector = _q_p_dot_cmd.tail(_n_jnts_robot);
            _robot->setVelocityReference(_auxiliary_vector);
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
    
    _robot->move();
}

void JumpReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/JumpReplayerRt", opt); // date-time automatically appended
    }
    else
    {
        if (!_is_sim)
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        else
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("stop_stiffness", _stop_stiffness);
    _dump_logger->add("stop_damping", _stop_damping);
    _dump_logger->add("is_sim", int(_is_sim));
    
    _dump_logger->add("send_pos_ref", int(_send_pos_ref));
    _dump_logger->add("send_vel_ref", int(_send_vel_ref));
    _dump_logger->add("send_eff_ref", int(_send_eff_ref));

    _dump_logger->add("driver_temp_threshold", _driver_temp_threshold);

    // _dump_logger->create("plugin_time", 1);
    _dump_logger->create("jump_replay_times", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_time", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_driver_temp", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("is_drivers_temp_ok", 1, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_meas", _n_jnts_robot), 1, _matlogger_buffer_size;
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);

}

void JumpReplayerRt::add_data2dump_logger()
{

    if (_is_first_jnt_passive)
    { // remove first joint from logged commands
        _auxiliary_vector = _q_p_cmd.tail(_n_jnts_robot);
        _dump_logger->add("q_p_cmd", _auxiliary_vector);
        _auxiliary_vector = _q_p_dot_cmd.tail(_n_jnts_robot);
        _dump_logger->add("q_p_dot_cmd", _auxiliary_vector);
        _auxiliary_vector = _tau_cmd.tail(_n_jnts_robot);
        _dump_logger->add("tau_cmd", _auxiliary_vector);

    }
    else
    {

        _dump_logger->add("q_p_cmd", _q_p_cmd);
        _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd);
        _dump_logger->add("tau_cmd", _tau_cmd);

    }

    _dump_logger->add("replay_stiffness", _replay_stiffness);
    _dump_logger->add("replay_damping", _replay_damping);
    _dump_logger->add("meas_stiffness", _meas_stiffness);
    _dump_logger->add("meas_damping", _meas_damping);

    _dump_logger->add("meas_driver_temp", _meas_driver_temp);

    _dump_logger->add("is_drivers_temp_ok", _is_drivers_temp_ok);

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

    _dump_logger->add("replay_time", _loop_time);

}

void JumpReplayerRt::add_data2bedumped()
{
    
    // _dump_logger->add("plugin_time", _loop_time)

    if (_reduce_dumped_sol_size)
    {  // only adding data when replaying trajectory to save memory

        if (_traj_started && !_traj_finished)
        { // trajectory is being published
            
            if (_sample_index <= (_traj.get_n_nodes() - 1))
            {

                add_data2dump_logger();

            }

        }
    }
    else
    { // dump data at each loop cycle

         add_data2dump_logger();

    }
    
}

void JumpReplayerRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    _jump_now_srv = _ros->advertiseService(
        "jump_now_srv",
        &JumpReplayerRt::on_jump_msg_rcvd,
        this,
        &_queue);

    /* Publishers */
    awesome_leg::MatReplayerStatus replay_st_prealloc;
    replay_st_prealloc.approach_traj_finished = false;
    replay_st_prealloc.traj_finished = false;

    _replay_status_pub = _ros->advertise<awesome_leg::MatReplayerStatus>(
        "replay_status_node", 1, replay_st_prealloc);


}

int JumpReplayerRt::was_jump_signal_received()
{
    int res = -1;

    if (_jump)
    {
        _is_first_trigger = !_is_first_trigger;
        
        if (!_approach_traj_started && !_traj_started && _is_first_trigger)
        { // triggered by the first jump signal
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                "\n Initializing approach trajectory sequence...\n Please wait for the robot to stop, place it on the ground and clear the jumping area.\n");

            res = 1;

            _q_p_trgt_appr_traj = _q_p_ref.block(1, 0, _n_jnts_robot, 1); // target pos. for the approach traj

            _imp_traj_started = true; // start impedance traj
            _approach_traj_started = true;

            _q_p_init_appr_traj = _q_p_cmd; // setting initial approach traj. point
            // to last sent position command

//            _approach_traj_started = false; // the pluginwill wait for imp. traj to finish
//            // before starting the approach trajectory

            _ramp_strt_stiffness = _meas_stiffness;
            _ramp_strt_damping = _meas_damping;
        }
        
        if (_approach_traj_finished && _imp_traj_finished && !_traj_started && !_is_first_trigger)
        { // triggered by the second jump signal
            
            _traj_started = true; // send actual trajectory

            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                "\n Starting jump sequence! Please clear the jumping area!\n");

            res = 2;

            _sample_index = 0; // will start sending the loaded trajectory
            
            _dump_logger->add("jump_replay_times", _loop_time);
            
        }

    }

    return res;
}

bool JumpReplayerRt::on_jump_msg_rcvd(const awesome_leg::JumpNowRequest& req,
                    awesome_leg::JumpNowResponse& res)
{

    _jump = req.jump_now; // setting jump flag

    check_driver_temp_limits(); // deactivates jump if temperature of any of the
    // joint driver goes beyond the prescibed threshold

    std::string message;
    
    _jump_phase_state = was_jump_signal_received();

    res.message = (_jump_phase_state == 1) ? "Starting replaying of approach trajectory!" : "Starting replaying of jump trajectory!";

    res.success = true;

    return res.success; 
}

void JumpReplayerRt::load_opt_data()
{   

    _traj = TrajLoader(_mat_path + _mat_name, true, 0.0001, false);

    int n_traj_jnts = _traj.get_n_jnts();

    if(n_traj_jnts != _n_jnts_robot) 
    {
        jwarn("The loaded trajectory has {} joints, while the robot has {} .\n Make sure to somehow select the right components!!",
        n_traj_jnts, _n_jnts_robot);
    }

    double dt_opt = -1.0;

    _traj.get_opt_dt(dt_opt);

    if( (abs(dt_opt - _plugin_dt) > 0.000001) and !_resample)
    {
        jerror("The plugin is running at a rate of {} Hz, while the loaded data was generated at {} Hz.\n Please make sure to replay the trajectory at the same rate of the plugin(or resample it)!!",
        1.0/_plugin_dt, 1.0/ dt_opt);
    }

    if (_resample)
    { // resample input data at the plugin frequency (for now it sucks)

        _traj.resample(_plugin_dt, _q_p_ref, _q_p_dot_ref, _tau_ref, _f_cont_ref); // just brute for linear interpolation for now (for safety, better to always use the same plugin_dt as the loaded trajectory)

    }
    else
    { // load raw data without changes

        Eigen::MatrixXd dt_opt;

        _traj.get_loaded_traj(_q_p_ref, _q_p_dot_ref, _tau_ref, dt_opt, _f_cont_ref);

    }

    _takeoff_index = _traj.get_takeoff_index(); // takeoff index

}

void JumpReplayerRt::saturate_cmds()
{

    _robot->enforceJointLimits(_q_p_cmd);
    _robot->enforceVelocityLimit(_q_p_dot_cmd);
    _robot->enforceEffortLimit(_tau_cmd);

}

void JumpReplayerRt::check_driver_temp_limits()
{

    bool were_drivers_temp_ok = _is_drivers_temp_ok; // getting previous value

    _is_drivers_temp_ok = (_meas_driver_temp.array() < _driver_temp_threshold.array()).all();

    if(!_is_drivers_temp_ok)
    {
        _jump = false; // do not start the jumping sequence
        // even if a positive jumping command was received

        jhigh().jprint(fmt::fg(fmt::terminal_color::red),
                   "\n Plugin driver temperature threshold exceeded. Disabling jump sequence...\n");

    }

    if(!were_drivers_temp_ok && _is_drivers_temp_ok)
    { // temperatures went under the threshold --> we can jump again
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Driver temperatures went under the plugin threshold. We can start jumping again... \n");

        reset_flags(); // resetting flags so that we start from the beginning of the jump pipeline
    }
}

void JumpReplayerRt::ramp_imp_smoothly()
{

    double phase = _smooth_imp_time / _imp_ramp_time;

    _ramp_stiffness = _peisekah_utils.compute_peisekah_vect_val(phase, _ramp_strt_stiffness, _replay_stiffness);

    _ramp_damping = _peisekah_utils.compute_peisekah_vect_val(phase, _ramp_strt_damping, _replay_damping);

    _stiffness_setpoint = _ramp_stiffness; 
    _damping_setpoint = _ramp_damping;

}

void JumpReplayerRt::set_approach_trajectory()
{

    double phase = _approach_traj_time / _approach_traj_exec_time; // phase ([0, 1] inside the approach traj)

    _q_p_cmd = _peisekah_utils.compute_peisekah_vect_val(phase, _q_p_init_appr_traj, _q_p_trgt_appr_traj);
    
}

void JumpReplayerRt::pub_replay_status()
{
    auto status_msg = _replay_status_pub->loanMessage();
    status_msg->msg().approach_traj_finished = _approach_traj_finished;
    status_msg->msg().traj_finished = _traj_finished;

    _replay_status_pub->publishLoaned(std::move(status_msg));
}

void JumpReplayerRt::set_trajectory()
{ // always called in each plugin loop
  // is made of a number of phases, each signaled by suitable flags
  // remember to increase the sample index at the end of each phase, 
  // if necessary

    if (_is_first_run)
    { // set impedance vals and pos ref to safe values at first plugin loop

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (first run) \n");
        }

        // setting commands to currently meas
        // state to avoid jumps upon plugin start
        _stiffness_setpoint = _meas_stiffness;
        _damping_setpoint = _meas_damping;

        _q_p_cmd = _q_p_meas;

    }

    if (_imp_traj_started && !_imp_traj_finished)
    { // still ramping (up) impedance
        
        if (_smooth_imp_time > _imp_ramp_time - 0.000001)
        {
            _imp_traj_finished = true; // finished ramping imp.

            _stiffness_setpoint = _replay_stiffness; 
            _damping_setpoint = _replay_damping;

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Finished ramping joint impedance \n");
        }
        else
        {
            ramp_imp_smoothly();

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (ramping impedance...) \n");
            }
        }

    }

    if (_approach_traj_started && !_approach_traj_finished)
    { // still publishing the approach trajectory

        if (_approach_traj_time > _approach_traj_exec_time - 0.000001)
        {
            _approach_traj_finished = true; // finished approach traj
            
            // start of trajectory replay is triggered by the callback in this case

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   std::string("\n Approach trajectory finished... ready to jump \n"));

        }
        else
        {
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (setting approach trajectory...) \n");
            }

            set_approach_trajectory();
        }

        _sample_index++; // incrementing loop counter
        
    }

    if (_traj_started && !_traj_finished && _jump)
    { // publish current trajectory sample
        
        if (_sample_index <= _takeoff_index)
        { // before takeoff
            
            // by default assign all commands anyway
            _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
            _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
            _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
            _f_contact_ref = _f_cont_ref.col(_sample_index);
            
            _stiffness_setpoint = _replay_stiffness; 
            _damping_setpoint = _replay_damping;

            saturate_cmds(); // saturate all cmds if they exceed limits
            // (position, velocity and efforts)

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (before takeoff) \n");
            }
        
        }

        if (_sample_index <= (_traj.get_n_nodes() - 1) && _sample_index > _takeoff_index)
        { // after the optimized takeoff phase
            
            if (_send_whole_traj)
            {
                _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
                _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
                _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
                _f_contact_ref = _f_cont_ref.col(_sample_index);
                
                _stiffness_setpoint = _replay_stiffness; // keep impedance high
                _damping_setpoint = _replay_damping;

            }
            else
            {
                _q_p_cmd = _q_p_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _q_p_dot_cmd = _q_p_dot_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _tau_cmd = _tau_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _f_contact_ref = _f_cont_ref.col(_takeoff_index);

                _stiffness_setpoint = _touchdown_stiffness; // low impedance
                _damping_setpoint = _touchdown_damping;

            }
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                    "\n (after takeoff) \n");
            }
            saturate_cmds(); // perform input torque saturation
            
        }

        if (_sample_index > (_traj.get_n_nodes() - 1))
        { // reached the end of the trajectory
            
            if (_send_whole_traj)
            { // set impedance to low value (ideally here we are in flight phase if
              // using the trajectory optimized up to the apex)

                _stiffness_setpoint = _touchdown_stiffness; // low impedance
                _damping_setpoint = _touchdown_damping;
            }

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (trajectory end)\n");
            }
            _traj_finished = true;
            _sample_index = 0; // reset publish index (will be used to publish the loaded trajectory)
            
        }

        _sample_index++; // incrementing loop counter

    }

    if (_traj_finished && _jump)
    { // finished publishing trajectory

        _pause_started = true;

        if (!_pause_finished)
        { // do nothing in this control loop

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                    "\n (in pause) \n");
            }
        }
        else 
        {

            reset_flags(); // reset flags

            _jump = false; // directly trigger next jump sequence

            _q_p_safe_cmd = _q_p_meas; // keep position reference to currently measured state

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                    "\n (pause ended) \n");
            }
        }

        _stiffness_setpoint = _touchdown_stiffness; 
        _damping_setpoint = _touchdown_damping;

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n Finished publishing \n");
        }
        
        _sample_index++; // incrementing loop counter
    }

    if (!_jump)
    {
        _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
        _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n Waiting for commands... \n");
        }
    }

}

bool JumpReplayerRt::on_initialize()
{ 
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();

    init_vars();

    _robot->getEffortLimits(_effort_lims);

    init_nrt_ros_bridge();
    
    // Initializing CartesIO solver, ros server and spawning the non rt thread
    // init_cartesio_solver();
    create_ros_api();

    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    _peisekah_utils = PeisekahTrans();

    return true;
    
}

void JumpReplayerRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers
    
    _stiffness_setpoint = _replay_stiffness;
    _damping_setpoint = _replay_damping;

    // setting the control mode to effort + velocity + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() +
            ControlMode::Damping());

    update_state(); // read current jnt positions and velocities

    // Move on to run()
    start_completed();
    
}

void JumpReplayerRt::run()
{  
    int i = 0;

    update_state(); // update all necessary states

    _queue.run();

    set_trajectory();

    pub_replay_status();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    send_cmds(); // send commands to the robot

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

}

void JumpReplayerRt::on_stop()
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

    _is_first_run = true;

    reset_flags();

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void JumpReplayerRt::stopping()
{

    stop_completed();

}

void JumpReplayerRt::on_abort()
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

void JumpReplayerRt::on_close()
{
    jinfo("Closing JumpReplayerRt");
}

XBOT2_REGISTER_PLUGIN(JumpReplayerRt, jmp_replayer_rt)

