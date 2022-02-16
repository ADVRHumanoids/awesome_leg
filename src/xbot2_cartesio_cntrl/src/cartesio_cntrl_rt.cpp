#include "cartesio_cntrl_rt.h"

void CartesioCntrlRt::get_params_from_config()
{
    // Reading some paramters from XBot2 config. YAML file

    bool tau_tilde_found = getParam("~torque_bias", _tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", _urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", _srdf_path); // srdf_path specific to gravity compensator
    bool cartesio_path_found = getParam("~cartesio_yaml_path", _cartesio_path); // srdf_path specific to gravity compensator
    bool stiffness_found = getParam("~stiffness", _stiffness);
    bool damping_found = getParam("~damping", _damping);
    bool dt_found = getParam("~dt", _dt);
}

void CartesioCntrlRt::init_model_interface()
{
    // Initializing XBot2 ModelInterface
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path);
    xbot_cfg.set_srdf_path(_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    _model = XBot::ModelInterface::getModel(xbot_cfg); 
    _n_jnts_model = _model->getJointNum();

}

void CartesioCntrlRt::init_cartesio_solver()
{
    // Before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(_dt),
                _model);

    // Load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(_cartesio_path);

    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // We are finally ready to make the CartesIO solver "OpenSot"
    _solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, ctx);
}

void CartesioCntrlRt::update_state()
{
    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);    
    
    jwarn("Hip q_p:");
    jwarn(std::to_string(_q_p_meas[0]));
    jwarn("Knee q_p:");
    jwarn(std::to_string(_q_p_meas[1]));
    jwarn("\n");

    jwarn("Hip q_p_dot:");
    jwarn(std::to_string(_q_p_dot_meas[0]));
    jwarn("Knee q_p_dot:");
    jwarn(std::to_string(_q_p_dot_meas[1]));
    jwarn("\n");

    // Updating the model with the measurements
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas);
    _model->update();

    _solver->update(_time, _dt);
    
}

void CartesioCntrlRt:: compute_joint_efforts()
{
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas); 
    _model->update();
    _model->getJointAcceleration(_q_p_ddot_ci); 
    // _model->setJointAcceleration(_q_p_ddot_ci); 
    // _model->update();
    _model->computeInverseDynamics(_effort_command);
}

bool CartesioCntrlRt::on_initialize()
{
    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    // Initializing XBot2 model interface using the read parameters 
    init_model_interface();

    // Setting robot control mode, stiffness and damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping()); // setting the control mode to effort + stiffness + damping
    _n_jnts_robot = _robot->getJointNum();

    // Initializing CartesIO solver
    init_cartesio_solver();

    return true;
}

void CartesioCntrlRt::starting()
{
    // initializing time (used for interpolation of trajectories inside CartesIO)
    _time = 0;

    // "sensing" the robot
    _robot->sense();

    // Update the model with the current robot state
    _model->update(); 

    // Reset CartesIO solver
    _solver->reset(_time);

    // Move on to run()
    start_completed();
}

void CartesioCntrlRt::run()
{   
    // Update the measured state and update CartesIO solver
    update_state();

    // Read the joint effort computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    // compute_joint_efforts();

    jwarn("Hip effort:");
    jwarn(std::to_string(_effort_command[0]));
    jwarn("Knee effort:");
    jwarn(std::to_string(_effort_command[1]));
    jwarn("\n");

    // Set the effort commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time
    _time += _dt;
}

XBOT2_REGISTER_PLUGIN(CartesioCntrlRt, cartesio_cntrl_rt)