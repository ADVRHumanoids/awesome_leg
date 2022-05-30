#include "mat_replayer_rt.h"

#include <math.h> 

#include "plugin_utils.h"

void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 
    _srdf_path = getParamOrThrow<std::string>("~srdf_path"); 
    _mat_path = getParamOrThrow<std::string>("~mat_path");  
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");

    _cntrl_mode: getParamOrThrow<std::VectorXd>("~cntrl_mode");
    _replay_stiffness: getParamOrThrow<std::VectorXd>("~cntrl_mode"); 
    _replay_damping: getParamOrThrow<std::VectorXd>("~cntrl_mode");
    _replay_dt: getParamOrThrow<double>("~cntrl_mode");  
    _looped_traj: getParamOrThrow<bool>("~cntrl_mode");
    _traj_pause_time: getParamOrThrow<double>("~cntrl_mode");

}

void MatReplayerRt::init_model_interface()
{
    
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path);
    xbot_cfg.set_srdf_path(_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // Initializing XBot2 ModelInterface for the rt thread
    _model = XBot::ModelInterface::getModel(xbot_cfg); 
    _n_jnts_model = _model->getJointNum();

}

bool MatReplayerRt::load_opt_data()
{   
    XBot::MatLogger2::Options opts;
    opts.load_file_from_path = true; // enable reading

    _logger = XBot::MatLogger2::MakeLogger(_mat_path, opts);

    int slices; // not needed -> we should make it an optional argument to readvar
    bool q_p_read_ok = _logger->readvar("q_p", _q_p_ref, slices);
    bool q_p_dot_read ok = _logger->readvar("q_p_dot", _q_p_dot_ref, slices);
    bool tau_read_ok = _logger->readvar("tau", _tau_ref, slices);
    bool dt_read_ok = _logger->readvar("dt_opt", _dt_opt, slices);

    _n_samples = _q_p_ref.cols() - 1; // the first sample is the initial condition, so is removed. This way state and input targets have the same size

    if (q_p_read_ok && q_p_dot_read && tau_read_ok && dt_read_ok)
    { // all variables read successfully
        return true;
    }
    else
    { // at leas one reading failed
        return false;
    }

}

void CartesioEllRt::saturate_input()
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

void MatReplayerRt::send_approach_traj()
{

}

void MatReplayerRt::send_trajectory()
{
    _robot->setPositionReference(_q_p_meas);
}

bool MatReplayerRt::on_initialize()
{   
    get_params_from_config();
    _dt = getPeriodSec();
    bool data_loaded_ok = load_opt_data();

    init_model_interface();

    // create a nodehandle with namespace equal to
    // the plugin name
    ros::NodeHandle nh(getName());
    // use it to create our 'RosSupport' class
    // note: this must run *after* ros::init, which
    // at this point is guaranteed to be true
    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    _ell_traj_srv = _ros->advertiseService(
        "my_ell_traj_srvc",
        &CartesioEllRt::on_ell_traj_recv_srv,
        this,
        &_queue);

    _model->getEffortLimits(_effort_lims);
    
    return true;
}

void MatReplayerRt::starting()
{
    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // Move on to run()
    start_completed();

}

void MatReplayerRt::run()
{  
   if (_first_run)
   {
       _approach_traj_started = true;

   }
   else{
       
   }
   
   // Increment samples index
    _sample_index += 1;
    
    if (_sample_index >= _t_exec_traj)
    {
        _time = _time - _t_exec_traj;
    }
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
    _logger.reset();
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