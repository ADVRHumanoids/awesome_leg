#include "mat_replayer_rt.h"

#include <math.h> 

#include "plugin_utils.h"

void MatReplayerRt::update_clocks()
{
    // Update time(s)
    _loop_time += _plugin_dt;
    
    if (_loop_time >= _t_exec_traj)
    {
        _loop_time = _loop_time - _t_exec_traj;
    }
}
void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 

    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");

    _cntrl_mode: getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");
    _replay_stiffness: getParamOrThrow<Eigen::VectorXd>("~cntrl_mode"); 
    _replay_damping: getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");
    _replay_dt: getParamOrThrow<double>("~cntrl_mode");  
    _looped_traj: getParamOrThrow<bool>("~cntrl_mode");
    _traj_pause_time: getParamOrThrow<double>("~cntrl_mode");

}

// void MatReplayerRt::init_model_interface()
// {
    
//     XBot::ConfigOptions xbot_cfg;
//     xbot_cfg.set_urdf_path(_urdf_path);
//     xbot_cfg.set_srdf_path(_srdf_path);
//     xbot_cfg.generate_jidmap();
//     xbot_cfg.set_parameter("is_model_floating_base", false);
//     xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

//     // Initializing XBot2 ModelInterface for the rt thread
//     _model = XBot::ModelInterface::getModel(xbot_cfg); 
//     _n_jnts_model = _model->getJointNum();

// }

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

bool MatReplayerRt::load_opt_data()
{   
    
    // // Reading data using a mat file

    // XBot::MatLogger2::Options opts;
    // opts.load_file_from_path = true; // enable reading

    // _load_logger = XBot::MatLogger2::MakeLogger(_mat_path, opts);

    // int slices; // not needed -> we should make it an optional argument to readvar
    // bool q_p_read_ok = _load_logger->readvar("q_p", _q_p_ref, slices);
    // bool q_p_dot_read ok = _load_logger->readvar("q_p_dot", _q_p_dot_ref, slices);
    // bool tau_read_ok = _load_logger->readvar("tau", _tau_ref, slices);
    // bool dt_read_ok = _load_logger->readvar("dt_opt", _dt_opt, slices);

    // _n_samples = _q_p_ref.cols() - 1; // the first sample is the initial condition, so is removed. This way state and input targets have the same size

    // if (q_p_read_ok && q_p_dot_read && tau_read_ok && dt_read_ok)
    // { // all variables read successfully
    //     return true;
    // }
    // else
    // { // at leas one reading failed
    //     return false;
    // }

    // Temporary reading from CSV files

    std::string data_path = _mat_path;

    _q_p_ref = plugin_utils::openData(data_path + "q_p.csv");
    _q_p_dot_ref = plugin_utils::openData(data_path + "q_p_dot.csv");
    _tau_ref = plugin_utils::openData(data_path + "tau.csv");
    _dt_opt = plugin_utils::openData(data_path + "dt_opt.csv");
    
    auto n_traj_jnts = _q_p_ref.rows();
    _n_samples = _q_p_ref.cols();

    // Here some checks on dimension consistency should be added

    if(n_traj_jnts != _n_jnts_model) 
    {
        jerror("The loaded trajectory has {} joints, while the robot has {} .", n_traj_jnts, _n_jnts_model);
    }

    _nominal_traj_dt = _dt_opt(0);
    _t_exec_traj = _dt_opt(0) * (_n_samples - 1); 
    _traj_time_vector = Eigen::VectorXd::Zero(_n_samples);

    for (int i = 0; i < _n_samples - 1; i++) // populating trajectory time vector
    {
        _traj_time_vector[i + 1] = _traj_time_vector[i] + _nominal_traj_dt;  
    }   

    return true;

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

void MatReplayerRt::sample_trajectory()
{

}

void MatReplayerRt::send_trajectory()
{
    // _robot->setPositionReference();

    _traj_finished = true;
}

bool MatReplayerRt::on_initialize()
{   
    get_params_from_config(); // load params from yaml file
    
    _plugin_dt = getPeriodSec();

    _n_jnts_model = _robot->getJointNum();

    _robot->getEffortLimits(_effort_lims);

    bool data_loaded_ok = load_opt_data(); // load trajectory from file

    // init_model_interface();

    init_dump_logger();

    init_nrt_ros_bridge();

    return true;
}

void MatReplayerRt::starting()
{
    _first_run = true; // reset flag in case the plugin is run multiple times
    _loop_time = 0.0; // reset loop time clock

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    _robot->setStiffness(_replay_stiffness);
    _robot->setDamping(_replay_damping);

    // Move on to run()
    start_completed();

}

void MatReplayerRt::run()
{  
    if (_first_run)
    { // first time entering the control loop

        _approach_traj_started = true; // flag signaling the start of the approach trajectory

    }

    if (_approach_traj_finished)
    { // start publishing the loaded trajectory
        _traj_started = true;
    }

    if (_traj_finished)
    { // finished publishing trajectory

        if (_looped_traj)
        { // restart publishing trajectory

            _traj_finished = false; // reset flag

        }

    }
    {
        send_trajectory();
    }

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

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