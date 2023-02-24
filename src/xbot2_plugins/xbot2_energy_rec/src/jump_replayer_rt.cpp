#include "jump_replayer_rt.h"

#include <math.h> 

void JumpReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _approach_traj_time = 0.0;

    _smooth_imp_time = 0.0;

    _go2touchdown_time = 0.0;

}

void JumpReplayerRt::reset_clocks()
{

    _approach_traj_time = 0.0;

    _smooth_imp_time = 0.0;

    _go2touchdown_time = 0.0;

}

void JumpReplayerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;


    if(_ramp_imp)
    {
        _smooth_imp_time += _plugin_dt;
    }

    if(_go2takeoff_config)
    {
        _approach_traj_time += _plugin_dt;
    }

    if(_go2landing_config)
    {
        _go2touchdown_time += _plugin_dt;
    }


    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

    if(_approach_traj_finished)
    {
        _approach_traj_time = _approach_traj_exec_time;
    }

    if(_imp_traj_finished)
    {
        _smooth_imp_time = _imp_ramp_time;
    }

    if(_landing_config_reached)
    {
        _go2touchdown_time = _go2touchdown_exec_time;
    }
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

    _auxiliary_vector = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_cmd_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_cmd_vect = std::vector<double>(_n_jnts_robot);
    _tau_cmd_vect = std::vector<double>(_n_jnts_robot);

    _f_contact_ref = Eigen::VectorXd::Zero(3);

    _f_contact_ref_vect = std::vector<double>(3);;

    _landing_config = Eigen::MatrixXd::Zero(_n_jnts_robot, 1);
    _touchdown_damping = Eigen::MatrixXd::Zero(_n_jnts_robot, 1);
    _touchdown_stiffness=  Eigen::MatrixXd::Zero(_n_jnts_robot, 1);

    _min_ramp_exec_time = Eigen::MatrixXd::Zero(_n_jnts_robot, 1);

    _stiffness_setpoint = Eigen::VectorXd::Zero(_n_jnts_robot);
    _damping_setpoint =  Eigen::VectorXd::Zero(_n_jnts_robot);

}

void JumpReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _takeoff_traj_path = getParamOrThrow<std::string>("~takeoff_traj_path");
    _takeoff_traj_name = getParamOrThrow<std::string>("~takeoff_traj_name");

    _landing_config_path = getParamOrThrow<std::string>("~landing_config_path");
    _landing_config_name = getParamOrThrow<std::string>("~landing_config_name");

    _verbose = getParamOrThrow<bool>("~verbose");

    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness");
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");

    _go2touchdown_exec_time = getParamOrThrow<double>("~go2touchdown_exec_time");

    _go2touchdown_config_auto = getParamOrThrow<bool>("~go2touchdown_config_auto");

//    _touchdown_stiffness = getParamOrThrow<Eigen::VectorXd>("~touchdown_stiffness");
//    _touchdown_damping = getParamOrThrow<Eigen::VectorXd>("~touchdown_damping");

//    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
//    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

//    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive");

//    _resample = getParamOrThrow<bool>("~resample");

//    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
//    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");

//    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
//    _imp_ramp_time = getParamOrThrow<double>("~imp_ramp_time");



//    _send_pos_ref = getParamOrThrow<bool>("~send_pos_ref");
//    _send_vel_ref = getParamOrThrow<bool>("~send_vel_ref");
//    _send_eff_ref = getParamOrThrow<bool>("~send_eff_ref");

//    _reduce_dumped_sol_size = getParamOrThrow<bool>("~reduce_dumped_sol_size");

//    _send_whole_traj = getParamOrThrow<bool>("~send_whole_traj");


//    _resample_err_tolerance = getParamOrThrow<double>("~resample_err_tolerance");

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

void JumpReplayerRt::is_dummy(std::string dummy_string = "dummy")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t dummy_found = _hw_type.find(dummy_string);


    if (dummy_found != std::string::npos) { // we are running the plugin in dummy mode

        _is_dummy = true;
    }
    else // we are running on the real robot
    {
        _is_dummy = false;
    }

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

}

void JumpReplayerRt::send_cmds()
{
    // always set impedance to the last setpoint to avoid issues
    _robot->setStiffness(_stiffness_setpoint); 
    _robot->setDamping(_damping_setpoint);

    if (_is_first_jnt_passive)
    { // send the last _n_jnts_robot components

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
        if (_send_eff_ref)
        {
            _auxiliary_vector = _tau_cmd.tail(_n_jnts_robot);
            _robot->setEffortReference(_auxiliary_vector);
        }

    }
    else{

        if(_send_pos_ref)
        {  

            _robot->setPositionReference(_q_p_cmd);
        }

        if(_send_vel_ref)
        {  

            _robot->setVelocityReference(_q_p_dot_cmd);
        }

        if (_send_eff_ref)
        {
            _robot->setEffortReference(_tau_cmd);
        }

    }
    
    _robot->move(); // send commands to the robot
}

void JumpReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger(_dump_path, opt); // date-time automatically appended
    }
    else
    {
        if (!_is_sim)
        {
            _dump_logger = MatLogger2::MakeLogger(_takeoff_traj_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        else
        {
            _dump_logger = MatLogger2::MakeLogger(_takeoff_traj_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));
    
    _dump_logger->add("send_pos_ref", int(_send_pos_ref));
    _dump_logger->add("send_vel_ref", int(_send_vel_ref));
    _dump_logger->add("send_eff_ref", int(_send_eff_ref));

    _dump_logger->add("go2touchdown_exec_time", _go2touchdown_exec_time);

    _dump_logger->create("plugin_time", 1);
    _dump_logger->create("jump_replay_times", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_time", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->add("touchdown_stiffness", _touchdown_stiffness);
    _dump_logger->add("touchdown_damping", _touchdown_damping);
    _dump_logger->create("meas_driver_temp", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("ramp_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("ramp_damping", _n_jnts_robot, 1, _matlogger_buffer_size);

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

    _dump_logger->add("plugin_time", _loop_time);

    _dump_logger->add("replay_stiffness", _replay_stiffness);
    _dump_logger->add("replay_damping", _replay_damping);
    _dump_logger->add("meas_stiffness", _meas_stiffness);
    _dump_logger->add("meas_damping", _meas_damping);

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

    _dump_logger->add("replay_time", _loop_time);


    _dump_logger->add("ramp_stiffness", _ramp_stiffness);
    _dump_logger->add("ramp_damping", _ramp_damping);

}

void JumpReplayerRt::add_data2bedumped()
{
    

    if (_reduce_dumped_sol_size)
    {  // only adding data when replaying trajectory to save memory

        if (_perform_takeoff)
        {

            add_data2dump_logger();

        }
    }
    else
    { // dump data at each loop cycle and also during takeoff config approaching

         add_data2dump_logger();

    }
    
}

bool JumpReplayerRt::on_go2takeoff_config_received(const awesome_leg::Go2TakeoffConfigRequest & req,
                                                   awesome_leg::Go2TakeoffConfigResponse& res)
{

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                      "\nJumpReplayerRt: received go2takeoff_config signal: {}\n", req.go2takeoff_config);
    }
    
    bool state_changed = _go2takeoff_config != req.go2takeoff_config;

    bool result = state_changed && (!_perform_takeoff && !_ramp_imp && !_go2landing_config)? true : false;

    if(result)
    { // we only assign the flag if we're not performing the takeoff or ramping impedance
        _go2takeoff_config = req.go2takeoff_config;

    }

    if(_go2takeoff_config && result)
    {
        _q_p_init_appr_traj = _q_p_cmd; // setting initial approach traj. point
        // to last sent position command

        reset_clocks();

        _approach_traj_finished = false;
    }
    if(!_go2takeoff_config)
    {
        reset_clocks();
        _approach_traj_finished = false;
    }

    res.success = result;

    return result;

}

bool JumpReplayerRt::on_perform_takeoff_received(const awesome_leg::PerformTakeoffRequest& req,
                                                 awesome_leg::PerformTakeoffResponse& res)
{
    
    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                        "\nJumpReplayerRt: received perform_takeoff signal: {}\n", req.perform_takeoff);
    }

    bool state_changed = _perform_takeoff != req.perform_takeoff;

    bool result = state_changed && (!_go2takeoff_config  && !_ramp_imp && !_go2landing_config)? true : false;

    if(result)
    { // we only assign the flag if we're not performing the approach trajectory or ramping impedance
        _perform_takeoff = req.perform_takeoff;

    }

    if(_perform_takeoff && result)
    {
        _sample_index = 0; // will start sending the loaded trajectory from
        // the first sample

        reset_clocks();

        _traj_finished = false;

    }
    if(!_perform_takeoff)
    {
        reset_clocks();
        _traj_finished = false;
    }

    res.success = result;

    return result;

}

bool JumpReplayerRt::on_ramp_jnt_imp_received(const awesome_leg::RampJntImpRequest& req,
                                              awesome_leg::RampJntImpResponse& res)
{
    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                           "\nJumpReplayerRt: received ramp_imp signal: {}\n", req.ramp_imp);
    }
    
    bool state_changed = _ramp_imp != req.ramp_imp;

    bool result = state_changed && (!_go2takeoff_config && !_perform_takeoff && !_go2landing_config)? true : false;

    if(result)
    { // we only assign the flag if we're not performing the approach trajectory, nor the jumping trajectory
        _ramp_imp = req.ramp_imp;

    }

    if(_ramp_imp && result)
    { // we set the ramp start at the last measured stiffness

        reset_clocks();

        _ramp_strt_stiffness = _meas_stiffness;
        _ramp_strt_damping = _meas_damping;

        _imp_traj_finished = false;
    }
    if(!_ramp_imp)
    {

        reset_clocks();
        _imp_traj_finished = false;
    }

    res.success = result;

    return result;

}

bool JumpReplayerRt::on_go2landing_config_received(const awesome_leg::Go2LandingConfigRequest& req,
                                              awesome_leg::Go2LandingConfigResponse& res)
{

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                           "\nJumpReplayerRt: received ramp_imp signal: {}\n", req.go2landing_config);
    }

    bool state_changed = _go2landing_config != req.go2landing_config;

    bool result = state_changed && (!_go2takeoff_config && !_perform_takeoff && !_ramp_imp) ? true : false;

    if(result)
    { // we only assign the flag if we're not performing the approach trajectory, nor the jumping trajectory
        _go2landing_config = req.go2landing_config;

    }

    if(_go2landing_config && result)
    {
        reset_clocks();

        _ramp_strt_stiffness = _meas_stiffness; // we prepare to ramp impedances from currently measured value
        _ramp_strt_damping = _meas_damping;

        _q_p_init_appr_traj = _q_p_cmd; // we also prepare to ramp the joint reference position

        _landing_config_reached = false;
    }
    if(!_go2landing_config)
    {

        reset_clocks();

        _landing_config_reached = false;
    }

    res.success = result;

    return result;

}

void JumpReplayerRt::init_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service servers */
    _go2takeoff_config_srvr = _ros->advertiseService(
        "go2takeoff_srvr",
        &JumpReplayerRt::on_go2takeoff_config_received,
        this,
        &_queue);

    _perform_takeoff_srvr = _ros->advertiseService(
        "perform_takeoff_srvr",
        &JumpReplayerRt::on_perform_takeoff_received,
        this,
        &_queue);

    _ramp_jnt_imp_srvr = _ros->advertiseService(
        "ramp_jnt_imp_srvr",
        &JumpReplayerRt::on_ramp_jnt_imp_received,
        this,
        &_queue);

    _go2lading_config_srvr = _ros->advertiseService(
        "go2touchdown_config_srvr",
        &JumpReplayerRt::on_go2landing_config_received,
        this,
        &_queue);

    /* Publishers */
    awesome_leg::MatReplayerStatus replay_st_prealloc;
    replay_st_prealloc.approach_traj_finished = false;
    replay_st_prealloc.traj_finished = false;

    _replay_status_pub = _ros->advertise<awesome_leg::MatReplayerStatus>(
        "replay_status_node", 1, replay_st_prealloc);


}

void JumpReplayerRt::load_opt_data()
{   

    std::string jump_traj_full_path = _takeoff_traj_path + _takeoff_traj_name;

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## Trying to load trajectory .mat file @ {}\n \n", jump_traj_full_path);

    _traj = TrajLoader(jump_traj_full_path, true, _resample_err_tolerance, false);

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


    std::string landing_config_fullpath = _landing_config_path + _landing_config_name;

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## Trying to load landing configuration .mat file @ {}\n \n", landing_config_fullpath);


    XBot::MatLogger2::Options opts;
    opts.load_file_from_path = true; // enable reading
    auto _landing_config_logger = XBot::MatLogger2::MakeLogger(landing_config_fullpath, opts);

    int slices; // not needed, used just to call the method properly
    bool stiff_read_ok = _landing_config_logger->readvar(_landing_stiffness_varname, _touchdown_stiffness_aux, slices); // here fix _dt_opt (should change to MatrixXd)
    bool damping_read_ok = _landing_config_logger->readvar(_landing_damping_varname, _touchdown_damping_aux, slices); // here fix _dt_opt (should change to MatrixXd)
    bool config_read_ok = _landing_config_logger->readvar(_landing_config_varname, _landing_config_aux, slices); // here fix _dt_opt (should change to MatrixXd)

    for(int i = 0; i < _n_jnts_robot; i++)
    {// we map values to VectorXd to avoid run time resize/non-rt-safe stuff
        _touchdown_damping(i) = _touchdown_damping_aux(i, 1);
        _touchdown_stiffness(i) = _touchdown_stiffness_aux(i, 1);
        _landing_config(i) = _landing_config_aux(i, 1);
    }

    if (!stiff_read_ok)
    { // reading failed
        throw std::runtime_error(std::string("Failed to read optimal landing stiffness from mat database at ") + landing_config_fullpath);
    }
    if (!damping_read_ok)
    { // reading failed
        throw std::runtime_error(std::string("Failed to read optimal landing damping from mat database at ") + landing_config_fullpath);
    }
    if (!config_read_ok)
    { // reading failed
        throw std::runtime_error(std::string("Failed to read optimal landing configuraton from mat database at ") + landing_config_fullpath);
    }
}

void JumpReplayerRt::saturate_cmds()
{

    _robot->enforceJointLimits(_q_p_cmd);
    _robot->enforceVelocityLimit(_q_p_dot_cmd);
    _robot->enforceEffortLimit(_tau_cmd);

}

void JumpReplayerRt::ramp_jnt_impedances()
{

    _phase = _smooth_imp_time / _imp_ramp_time;

    _peisekah_utils.compute_peisekah_vect_val(_phase, _ramp_strt_stiffness, _replay_stiffness, _ramp_stiffness);

    _peisekah_utils.compute_peisekah_vect_val(_phase, _ramp_strt_damping, _replay_damping, _ramp_damping);

    _stiffness_setpoint = _ramp_stiffness; 
    _damping_setpoint = _ramp_damping;

}

void JumpReplayerRt::ramp_towards_touchdown_config()
{
    _phase = _go2touchdown_time / _go2touchdown_exec_time;

    // stiffness transition
    _peisekah_utils.compute_peisekah_vect_val(_phase, _ramp_strt_stiffness, _touchdown_stiffness, _ramp_stiffness);
    // damping transition
    _peisekah_utils.compute_peisekah_vect_val(_phase, _ramp_strt_damping, _touchdown_damping, _ramp_damping);
    // joint configuration transition
    _peisekah_utils.compute_peisekah_vect_val(_phase, _q_p_init_appr_traj, _landing_config, _q_p_cmd);

    _stiffness_setpoint = _ramp_stiffness;
    _damping_setpoint = _ramp_damping;
}

void JumpReplayerRt::set_approach_trajectory()
{

    double phase = _approach_traj_time / _approach_traj_exec_time; // phase ([0, 1] inside the approach traj)

    _peisekah_utils.compute_peisekah_vect_val(phase, _q_p_init_appr_traj, _q_p_trgt_appr_traj, _q_p_cmd);
    
}

void JumpReplayerRt::set_cmds()
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

    // impedance ramping
    if (_ramp_imp)
    {
        if (_smooth_imp_time > _imp_ramp_time - 0.000001)
        { // finished ramping impedance

            _ramp_imp = false; // finished ramping imp.
            _imp_traj_finished = true;
            _imp_traj_started = false;

            _stiffness_setpoint = _replay_stiffness; 
            _damping_setpoint = _replay_damping;

            reset_clocks();

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Finished ramping joint impedance \n");
        }
        else
        {// ramp impedance

            if(!_imp_traj_started)
            {// triggered only the first time
                _imp_traj_started = true;
            }

            ramp_jnt_impedances();

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (ramping joint impedance...) \n");
            }
        }

    }

    // approach trajectory
    if (_go2takeoff_config)
    { // still publishing the approach trajectory

        if (_approach_traj_time > _approach_traj_exec_time - 0.000001)
        {
            _go2takeoff_config = false; // finished approach traj
            _approach_traj_finished = true;
            _approach_traj_started = false;

            reset_clocks();

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   std::string("\n Approach trajectory finished... ready to perform jumping trajectory \n"));

        }
        else
        {// set approach traj

            if(!_approach_traj_started)
            {// triggered only the first time
                _approach_traj_started = true;
            }
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (setting approach trajectory...) \n");
            }

            set_approach_trajectory();
        }
        
    }

    // actual jump trajectory
    if (_perform_takeoff)
    { // publish current trajectory sample
        
        if (_sample_index <= _takeoff_index)
        { // before takeoff
            
            if(!_traj_started)
            {// triggered only the first time
                _traj_started = true;
            }

            // by default assign all commands anyway
            _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
            _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
            _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
            _f_contact_ref = _f_cont_ref.col(_sample_index);
            
            _stiffness_setpoint = _replay_stiffness;
            _damping_setpoint = _replay_damping;

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (before nominal takeoff) \n");
            }
        
        }

        if (_sample_index <= (_traj.get_n_nodes() - 1) && _sample_index > _takeoff_index)
        { // after the takeoff phase

            if (_send_whole_traj)
            {// we continue sending references up to the apex
                _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
                _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
                _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
                _f_contact_ref = _f_cont_ref.col(_sample_index);

                _stiffness_setpoint = _replay_stiffness; // keep impedance high
                _damping_setpoint = _replay_damping;

                if (_verbose)
                {
                    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                        "\n (after nominal takeoff) \n");
                }

            }
            else
            {
                if (_verbose)
                {
                    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                           "\n (jump trajectory replay ended)\n");
                }

                _perform_takeoff = false;
                _traj_finished = true;
                _traj_started = false;

            }
            
        }

        if (_sample_index > (_traj.get_n_nodes() - 1))
        { // reached the end of the trajectory

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (jump trajectory replay ended)\n");
            }

            _perform_takeoff = false;
            _traj_finished = true;
            _traj_started = false;

            if(_go2touchdown_config_auto)
            {
                _ramp_strt_stiffness = _stiffness_setpoint; // we prepare to ramp impedances from the last sent command
                _ramp_strt_damping = _damping_setpoint;

                _q_p_init_appr_traj = _q_p_cmd; // we also prepare to ramp the joint reference position

                _landing_config_reached = false;

                _go2landing_config = true; // we directly go to next phase
            }

            reset_clocks();

            // we leave q_p to the latest send commands, but we set to zero velocity and torque reference for safety
            _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
            _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
        }

        _sample_index++; // incrementing trajectory index counter

    }

    // fast ramp towards the touchdown configuration and jnt impedance setpoints
    if (_go2landing_config)
    {
        if (_go2touchdown_time > _go2touchdown_exec_time - 0.000001)
        { // finished ramping impedance

            _go2landing_config = false; // finished ramping imp.
            _landing_config_reached = true;
            _landing_config_started = false;

            _stiffness_setpoint = _touchdown_stiffness;
            _damping_setpoint = _touchdown_damping;

            reset_clocks();

            _performed_jumps += 1; // this is the end of all the jumping phases --> we consider the jump sequence completed

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Finished ramping towards touchdown configuration \n");
        }
        else
        {// ramp impedance

            if(!_landing_config_started)
            {// triggered only the first time
                _landing_config_started = true;
            }

            ramp_towards_touchdown_config();

            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (ramping towards touchdown config...) \n");
            }
        }
    }

    saturate_cmds(); // always saturate commands
}

void JumpReplayerRt::pub_replay_status()
{
    auto status_msg = _replay_status_pub->loanMessage();

    status_msg->msg().imp_traj_started = _imp_traj_started;
    status_msg->msg().approach_traj_started = _approach_traj_started;
    status_msg->msg().traj_started = _traj_started;

    status_msg->msg().imp_traj_finished = _imp_traj_finished;
    status_msg->msg().approach_traj_finished = _approach_traj_finished;
    status_msg->msg().traj_finished = _traj_finished;

    status_msg->msg().send_pos = _send_pos_ref;
    status_msg->msg().send_vel = _send_vel_ref;
    status_msg->msg().send_eff = _send_eff_ref;

    status_msg->msg().performed_jumps = _performed_jumps;

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        _q_p_cmd_vect[i] = _q_p_cmd(i);
        _q_p_dot_cmd_vect[i] = _q_p_dot_cmd(i);
        _tau_cmd_vect[i]  = _tau_cmd(i);
    }

    for(int i = 0; i < 3; i++)
    {
        _f_contact_ref_vect[i] = _f_contact_ref(i);
    }

    status_msg->msg().pos_ref = _q_p_cmd_vect;
    status_msg->msg().vel_ref = _q_p_dot_cmd_vect;
    status_msg->msg().eff_ref = _tau_cmd_vect;

    status_msg->msg().f_cont_ref = _f_contact_ref_vect;

    _replay_status_pub->publishLoaned(std::move(status_msg));
}

bool JumpReplayerRt::on_initialize()
{ 
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _robot->getVelocityLimits(_jnt_vel_limits);

    init_vars();

    init_ros_bridge();

    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    _peisekah_utils = PeisekahTrans();

    _q_p_trgt_appr_traj = _q_p_ref.block(1, 0, _n_jnts_robot, 1); // target pos. for the approach traj

    return true;
    
}

void JumpReplayerRt::starting()
{

    init_dump_logger();

    init_clocks(); // initialize clocks timers

    // we let all types of commands pass
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() +
            ControlMode::Damping());

    // Move on to run()
    start_completed();
    
}

void JumpReplayerRt::run()
{  
    int i = 0;

    update_state(); // update all necessary states

    _queue.run(); // process server callbacks and update triggers' states, if necessary

    set_cmds(); // compute and set command references

    send_cmds(); // send commands to the robot (we always call this so that references are always provided to the robot). We need
    // to make sure references are as much continuous as possible

    pub_replay_status(); // publishes info from the plugin to ros and internal xbot2 topics

    add_data2dump_logger(); // add data for debugging purposes

    update_clocks(); // last, update the clocks (loop + any additional one). Each clock is incremented by a plugin dt

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

}

void JumpReplayerRt::on_stop()
{
    // Read the current state
//    _robot->sense();

    // Setting references before exiting
//    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    
//    _robot->setStiffness(_stop_stiffness);
//    _robot->setDamping(_stop_damping);
//    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
//    _robot->setPositionReference(_q_p_meas);

    // Sending references
//    _robot->move();

    _is_first_run = true;

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger->add("performed_jumps", _performed_jumps);
    _dump_logger.reset();
}

void JumpReplayerRt::stopping()
{

    stop_completed();

}

void JumpReplayerRt::on_abort()
{
    // Read the current state
//    _robot->sense();

    // Setting references before exiting
//    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

//    _robot->setStiffness(_stop_stiffness);
//    _robot->setDamping(_stop_damping);
//    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
//    _robot->setPositionReference(_q_p_meas);

    // Sending references
//    _robot->move();

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void JumpReplayerRt::on_close()
{
    jinfo("Closing JumpReplayerRt");
}

XBOT2_REGISTER_PLUGIN(JumpReplayerRt, jmp_replayer_rt)

