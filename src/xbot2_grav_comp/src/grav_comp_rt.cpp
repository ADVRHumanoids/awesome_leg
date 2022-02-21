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

    _n_jnts_robot = _robot->getJointNum();

    return true;
}

void GravCompRt::starting()
{
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
}

void GravCompRt::on_stop()
{
    // Reset control mode, stiffness and damping, so that the final position is kept at the end of the trajectory
    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

    // Setting references before exiting
    _robot->setPositionReference(_q_p);
    _robot->setVelocityReference(Eigen::VectorXd::Zero(2));

    _robot->move();
    
}

XBOT2_REGISTER_PLUGIN(GravCompRt, grav_comp_rt)