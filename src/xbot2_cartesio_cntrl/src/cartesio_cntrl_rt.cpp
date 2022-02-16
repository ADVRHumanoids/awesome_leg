#include "cartesio_cntrl_rt.h"

void CartesioCntrlRt:: compute_joint_efforts()
{
    this->_model->setJointPosition(this->_q_p_cart);
    this->_model->setJointVelocity(this->_q_p_dot_cart); // gravity compensation -> no q_p_dot
    this->_model->setJointAcceleration(this->_q_p_cart); // gravity compensation -> no q_p_ddot
    this->_model->update();
    this->_model->computeInverseDynamics(this->_effort_command);
}

bool CartesioCntrlRt::on_initialize()
{
    // Reading some paramters from YAML
    bool tau_tilde_found = getParam("~torque_bias", this->_tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", this->_urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", this->_srdf_path); // srdf_path specific to gravity compensator
    bool cartesio_path_found = getParam("~cartesio_yaml_path", this->_cartesio_path); // srdf_path specific to gravity compensator
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

    // Setting CartesIO solver

    Eigen::VectorXd qhome;
    this->_model->getRobotState("home", qhome);
    this->_model->setJointPosition(qhome);
    this->_model->update();

    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(this->_dt),
                this->_model);

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(this->_cartesio_path);
    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    this->_solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );

    return true;
}

void CartesioCntrlRt::starting()
{
    _robot->sense();
    
    start_completed();
}

void CartesioCntrlRt::run()
{   
    // Getting robot state
    this->_robot->getJointPosition(this->_q_p_meas);
    this->_robot->getJointVelocity(this->_q_p_dot_meas);    

    // Updating the model with the measurements
    this->_model->setJointPosition(this->_q_p_meas);
    this->_model->setJointVelocity(this->_q_p_dot_meas);
    this->_model->update();

    // Updating CartesIO solver
    this->_solver->update(this->_time, this->_dt);

    this->_model->getJointPosition(this->_q_p_cart);
    this->_model->getJointVelocity(this->_q_p_dot_cart);
    _model->getJointAcceleration(this->_q_p_ddot_cart);

    jwarn(std::to_string(this->_q_p_cart[0]));
    jwarn(std::to_string(this->_q_p_dot_cart[0]));
    jwarn(std::to_string(this->_q_p_ddot_cart[0]));
    jwarn("\n");

    // this->_q_p_ddot = Eigen::VectorXd::Zero(this->_n_jnts_model);

    // this->compute_joint_efforts(); // assigns to this->effort_command

    // this->_robot->setEffortReference(this->_effort_command + this->_tau_tilde);
    // this->_robot->setStiffness(this->_stiffness);
    // this->_robot->setDamping(this->_damping);

    // this->_robot->move(); // necessary to actually send any kind of reference to the robot

    this->_time += this->_dt;
}

XBOT2_REGISTER_PLUGIN(CartesioCntrlRt, cartesio_cntrl_rt)