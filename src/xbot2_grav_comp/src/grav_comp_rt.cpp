#include "grav_comp_rt.h"

void GravCompRt:: compute_joint_efforts()
{
    _model->setJointPosition(_q_p);
    _model->setJointVelocity(_q_p_dot); // gravity compensation -> no q_p_dot
    _model->setJointAcceleration(_q_p_ddot); // gravity compensation -> no q_p_ddot
    _model->update();
    _model->computeInverseDynamics(_effort_command);
}

bool GravCompRt::on_initialize()
{
    // Reading some paramters from YAML
    bool tau_tilde_found = getParam("~torque_bias", _tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", _urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", _srdf_path); // srdf_path specific to gravity compensator
    bool stiffness_found = getParam("~stiffness", _stiffness);
    bool damping_found = getParam("~damping", _damping);
    bool stop_stiffness_found = getParam("~stop_stiffness", _stop_stiffness);
    bool stop_damping_found = getParam("~stop_damping", _stop_damping);

    // Initializing XBot2 ModelInterface
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path);
    xbot_cfg.set_srdf_path(_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    _model = XBot::ModelInterface::getModel(xbot_cfg); 
    _n_jnts_model = _model->getJointNum();

    return true;
}

void GravCompRt::starting()
{
    // Creating a logger for post-processing (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the object is recreated)

    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/GravCompRt", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _robot->sense();

    // Setting robot control mode, stiffness and damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping()); // setting the control mode to effort + stiffness + damping
    
    start_completed();
}

void GravCompRt::run()
{
    
    _robot->getJointPosition(_q_p);
    _q_p_dot = Eigen::VectorXd::Zero(_n_jnts_model);
    _q_p_ddot = Eigen::VectorXd::Zero(_n_jnts_model);

    compute_joint_efforts(); // assigns to effort_command

    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    _robot->move(); // necessary to actually send any kind of reference to the robot

    _logger->add("computed_efforts", _effort_command);
    _logger->add("q_p", _q_p);

}

void GravCompRt::on_stop()
{   
    // Setting references before exiting

    _robot->sense();

    _robot->getJointPosition(_q_p);
    // Setting references before exiting
    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    _robot->setPositionReference(_q_p);
    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();
}

XBOT2_REGISTER_PLUGIN(GravCompRt, grav_comp_rt)