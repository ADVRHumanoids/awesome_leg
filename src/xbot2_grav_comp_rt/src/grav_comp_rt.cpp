#include "grav_comp_rt.h"

void GravCompRt:: compute_joint_efforts()
{
    this->_model->setJointPosition(this->_q_p);
    this->_model->setJointVelocity(Eigen::VectorXd::Zero(this->_n_jnts_model)); // gravity compensation -> no q_p_dot
    this->_model->setJointAcceleration(Eigen::VectorXd::Zero(this->_n_jnts_model)); // gravity compensation -> no q_p_ddot
    this->_model->update();
    this->_model->computeInverseDynamics(this->_effort_command);
}

bool GravCompRt::on_initialize()
{
    // Reading some paramters from YAML
    bool tau_tilde_found = getParam("~torque_bias", this->_tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", this->_urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", this->_srdf_path); // srdf_path specific to gravity compensator
    bool stiffness_found = getParam("~stiffness", this->_stiffness);
    bool damping_found = getParam("~damping", this->_damping);

    // Initializing XBot2 ModelInterface
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(this->_urdf_path);
    xbot_cfg.set_srdf_path(this->_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    this->_model = XBot::ModelInterface::getModel(xbot_cfg); 
    this->_n_jnts_model = this->_model->getJointNum();

    // Setting robot control mode, stiffness and damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping()); // setting the control mode to effort + stiffness + damping
    this->_n_jnts_robot = this->_robot->getJointNum();

    return true;
}

void GravCompRt::starting()
{
    _robot->sense();
    
    start_completed();
}

void GravCompRt::run()
{
    
    this->_robot->getJointPosition(this->_q_p);
    // this->_robot->getJointVelocity(this->_q_p_dot);
    // this->_robot->getJointAcceleration(this->_q_p_ddot);

    this->compute_joint_efforts(); // assigns to this->effort_command

    this->_robot->setEffortReference(this->_effort_command + this->_tau_tilde);
    this->_robot->setStiffness(this->_stiffness);
    this->_robot->setDamping(this->_damping);

    this->_robot->move(); // necessary to actually send any kind of reference to the robot
}

XBOT2_REGISTER_PLUGIN(GravCompRt, grav_comp_rt)