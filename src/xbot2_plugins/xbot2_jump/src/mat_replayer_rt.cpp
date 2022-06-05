#include "mat_replayer_rt.h"

#include <math.h> 

void MatReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
    _approach_traj_time = 0.0;
    _pause_time = 0.0;
}

void MatReplayerRt::update_clocks()
{
    // Update time(s)
    _loop_time += _plugin_dt;
    _approach_traj_time += _plugin_dt;
    _pause_time += _plugin_dt;
    
    // Reset timers, if necessary
    if (_loop_time >= _loop_reset_time)
    {
        _loop_time = _loop_time - _loop_reset_time;
    }
    if (_approach_traj_time >= _approach_traj_exec_time)
    {
        _approach_traj_time = _approach_traj_time - _approach_traj_exec_time;
    }
}

void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _math_name = getParamOrThrow<std::string>("~mat_name"); 
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");
    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
    _approach_traj_target = getParamOrThrow<Eigen::VectorXd>("~approach_traj_target");
    _cntrl_mode =  getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");
    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness"); 
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");
    _looped_traj = getParamOrThrow<bool>("~looped_traj");
    _traj_pause_time = getParamOrThrow<double>("~traj_pause_time");
    _send_pos_ref = getParamOrThrow<bool>("~send_pos_ref");
    _send_vel_ref = getParamOrThrow<bool>("~send_vel_ref");
    _send_effort_ref = getParamOrThrow<bool>("~send_effort_ref");

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

    auto dscrptn_files_cell = XBot::matlogger2::MatData::make_cell(2);
    dscrptn_files_cell[0] = _robot->getUrdfPath();
    dscrptn_files_cell[1] = _robot->getSrdfPath();
    _dump_logger->save("description_files", dscrptn_files_cell);

}

void MatReplayerRt::add_data2dump_logger()
{
    
    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("stop_stiffness", _stop_stiffness);
    _dump_logger->add("stop_damping", _stop_damping);

}

void MatReplayerRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    // /* Service server */
    // _ell_traj_srv = _ros->advertiseService(
    //     "my_ell_traj_srvc",
    //     &MatReplayerRt::on_ell_traj_recv_srv,
    //     this,
    //     &_queue);

}

void MatReplayerRt::load_opt_data()
{   

    _traj = plugin_utils::TrajLoader(_mat_path);
    int n_traj_jnts = _traj.get_n_jnts();

    if(n_traj_jnts != _n_jnts_model) 
    {
        jerror("The loaded trajectory has {} joints, while the robot has {} .", n_traj_jnts, _n_jnts_model);
    }

    _traj.resample(_plugin_dt, _q_p_ref, _q_p_dot_ref, _tau_ref);

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

        _robot->setPositionReference(_q_p_cmd);
    }
    
}

void MatReplayerRt::send_trajectory()
{
    if (_first_run)
    { // first time entering the control loop

        _approach_traj_started = true; // flag signaling the start of the approach trajectory

        _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping()); // setting control mode for the approach traj
        _robot->setStiffness(_replay_stiffness);
        _robot->setDamping(_replay_damping);
       
    }

    if (_traj_finished && _looped_traj)
    { // finished publishing trajectory

        if (_pause_time  < _traj_pause_time)
        { // do nothing in this control loop

        }
        else
        { // pause time is expired

            _sample_index = 0; // reset publishing index
            _traj_finished = false; // reset flag

        }
        
    }

    if (_approach_traj_started && !_approach_traj_finished)
    { // still publishing the approach trajectory

        send_approach_trajectory();
    }

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

            _q_p_dot_cmd = _q_p_ref.col(_sample_index);

            _tau_cmd = _tau_ref.col(_sample_index);

            if(_send_pos_ref)
            {
                _robot->setPositionReference(_q_p_cmd);
            }

            if(_send_vel_ref)
            {
                _robot->setVelocityReference(_q_p_dot_cmd);
            }

            if(_send_effort_ref)
            {
                _robot->setEffortReference(_tau_cmd);
            }
            
        }

    }

}

bool MatReplayerRt::on_initialize()
{   
    get_params_from_config(); // load params from yaml file
    
    _plugin_dt = getPeriodSec();

    _n_jnts_model = _robot->getJointNum();

    _robot->getEffortLimits(_effort_lims);

    load_opt_data(); // load trajectory from file

    init_dump_logger();

    init_nrt_ros_bridge();

    return true;
}

void MatReplayerRt::starting()
{
    _first_run = true; // reset flag in case the plugin is run multiple times
    _sample_index = 0; // resetting samples index, in case the plugin stopped and started again

    init_clocks(); // initialize clocks timers

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    _robot->setStiffness(_replay_stiffness);
    _robot->setDamping(_replay_damping);

    update_state(); // read current jnt positions and velocities

    compute_approach_traj(); // based on the current state, compute a smooth transition to the first trajectory position sample

    // Move on to run()
    start_completed();

}

void MatReplayerRt::run()
{  

    send_trajectory();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    if (_first_run == true)
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
    jinfo("Closing");
}

XBOT2_REGISTER_PLUGIN(MatReplayerRt, mat_replayer_rt)