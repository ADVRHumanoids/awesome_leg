#include "cartesio_imp_cntrl_rt.h"

void CartesioImpCntrlRt::get_params_from_config()
{
    // Reading some paramters from XBot2 config. YAML file

    bool tau_tilde_found = getParam("~torque_bias", _tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", _urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", _srdf_path); // srdf_path specific to gravity compensator
    bool cartesio_path_found = getParam("~cartesio_yaml_path", _cartesio_path); // srdf_path specific to gravity compensator
    bool stiffness_found = getParam("~stiffness", _stiffness);
    bool damping_found = getParam("~damping", _damping);
    bool stop_stiffness_found = getParam("~stop_stiffness", _stop_stiffness);
    bool stop_damping_found = getParam("~stop_damping", _stop_damping);
    bool t_exec_found = getParam("~t_exec", _t_exec);
    bool q_target_found = getParam("~q_target", _q_p_target);
}

void CartesioImpCntrlRt::init_model_interface()
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

void CartesioImpCntrlRt::init_cartesio_solver()
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

void CartesioImpCntrlRt::update_state()
{
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);    
    
    // Updating the model with the measurements
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas);
    _model->update();
    
}

void CartesioImpCntrlRt:: compute_joint_efforts()
{
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas); 
    _model->update();
    _model->getJointAcceleration(_q_p_ddot_ci); 
    _model->computeInverseDynamics(_effort_command);
}

bool CartesioImpCntrlRt::on_initialize()
{
    
    // Creating a logger for post-processing
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/CartesioCntrlRt_log", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Getting nominal control period from plugin method
    _dt = getPeriodSec();

    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    // Initializing XBot2 model interface using the read parameters 
    init_model_interface();

    // Setting robot control mode, stiffness and damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping()); // setting the control mode to effort + stiffness + damping
    _n_jnts_robot = _robot->getJointNum();

    // Initializing CartesIO solver
    init_cartesio_solver();

    // Getting tip cartesian task and casting it to cartesian (task)
    auto task = _solver->getTask("tip");
    
    // _cart_task_classic = std::dynamic_pointer_cast<CartesianTask>(task); // classical cartesian task
    _cart_task_int = std::dynamic_pointer_cast<InteractionTask>(task); // interaction cartesian task (exposes additional methods)

    _model->setJointPosition(_q_p_target);
    _model->update();
    _model->getPose("tip", _target_pose);
    
    return true;
}

void CartesioImpCntrlRt::starting()
{
    // initializing time (used for interpolation of trajectories inside CartesIO)
    _time = 0;

    // Update the model with the current robot state
    update_state();

    // Reset CartesIO solver
    _solver->reset(_time);

    // command reaching motion
    _cart_task_int->setPoseTarget(_target_pose, _t_exec);

    // Move on to run()
    start_completed();
}

void CartesioImpCntrlRt::run()
{   

    // Update the measured state
    update_state();
     
    // and update CartesIO solver
    _solver->update(_time, _dt);

    // Read the joint effort computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _robot->getJointEffort(_meas_effort);
    // compute_joint_efforts(); // getting efforts "manually"

    // Getting cartesian damping and stiffness for debugging purposes
    _cart_stiffness = _cart_task_int->getStiffness(); 
    _cart_damping = _cart_task_int->getDamping();
    
    // Set the effort commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time
    _time += _dt;

    _logger->add("meas_efforts", _meas_effort);
    _logger->add("computed_efforts", _effort_command);
    _logger->add("cartesian_stiffness", _cart_stiffness);
    _logger->add("cartesian_damping", _cart_damping);

}

void CartesioImpCntrlRt::on_stop()
{
    // Reset control mode, stiffness and damping, so that the final position is kept at the end of the trajectory
    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

    // Setting references before exiting
    _robot->setPositionReference(_q_p_meas);
    _robot->setVelocityReference(Eigen::VectorXd::Zero(2));

    _robot->move();
    
}

XBOT2_REGISTER_PLUGIN(CartesioImpCntrlRt, cartesio_imp_cntrl_rt)