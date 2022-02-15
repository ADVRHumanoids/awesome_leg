#include "cartesio_cntrl_rt.h"

void CartesioCntrlRt:: compute_joint_efforts()
{
    this->_model->setJointPosition(this->_q_p);
    this->_model->setJointVelocity(Eigen::VectorXd::Zero(this->_n_jnts_model)); // gravity compensation -> no q_p_dot
    this->_model->setJointAcceleration(Eigen::VectorXd::Zero(this->_n_jnts_model)); // gravity compensation -> no q_p_ddot
    this->_model->update();
    this->_model->computeInverseDynamics(this->_effort_command);

}

bool CartesioCntrlRt::on_initialize()
{
    // Reading some paramters from YAML
    bool PI_found = getParam("~PI", this->_PI); // compensated inertial parameters (not actually used)
    bool tau_tilde_found = getParam("~torque_bias", this->_tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", this->_urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", this->_srdf_path); // srdf_path specific to gravity compensator
    bool cartesio_yaml_path_found = getParam("~cartesio_yaml_path", this->_cartesio_yaml_path);
    bool stiffness_found = getParam("~stiffness", this->_stiffness);
    bool damping_found = getParam("~damping", this->_damping);
    bool dt_found = getParam("~dt", this->_dt);

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
    this->_robot->setStiffness(this->_stiffness);
    this->_robot->setDamping(this->_damping);

    // Eigen::VectorXd qhome;
    // this->_model->getRobotState("home", qhome);
    // this->_model->setJointPosition(qhome);
    // this->_model->update();

    // auto ik_pb_yaml = YAML::LoadFile(this->_cartesio_yaml_path);

    // auto ctx = std::make_shared<XBot::Cartesian::Context>(
    //             std::make_shared<XBot::Cartesian::Parameters>(this->_dt),
    //             this->_model);

    // ProblemDescription ik_pb(ik_pb_yaml, ctx); // load the ik problem given a yaml file

    // auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, ctx);

    return true;
}

void CartesioCntrlRt::starting()
{
    // jwarn("pippo1");
    _robot->sense();
    
    start_completed();
}

void CartesioCntrlRt::run()
{
    
    this->_robot->getJointPosition(this->_q_p);
    // this->_robot->getJointVelocity(this->_q_p_dot);
    // this->_robot->getJointAcceleration(this->_q_p_ddot);

    this->compute_joint_efforts(); // assigns to this->effort_command

    jwarn("hip");
    jwarn(std::to_string(this->_effort_command[0] + this->_tau_tilde[0]));
    jwarn("knee");
    jwarn(std::to_string(this->_effort_command[1] + + this->_tau_tilde[1]));
    jwarn("\n");

    this->_robot->setEffortReference(this->_effort_command + this->_tau_tilde);
    this->_robot->setStiffness(this->_stiffness);
    this->_robot->setDamping(this->_damping);

    this->_robot->move(); // necessary to actually send any kind of reference to the robot
    // _robot->setJointEffort();
    this->_last_sent_efforts = this->_effort_command; 
}

XBOT2_REGISTER_PLUGIN(GravCompRt, grav_comp_rt)