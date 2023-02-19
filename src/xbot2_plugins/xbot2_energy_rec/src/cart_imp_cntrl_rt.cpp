#include "cart_imp_cntrl_rt.h"
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

void CartImpCntrlRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void CartImpCntrlRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void CartImpCntrlRt::is_sim(std::string sim_string = "sim")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t sim_found = _hw_type.find(sim_string);

    if (sim_found != std::string::npos) { // we are running the plugin in simulation

        _is_sim = true;
    }
    else // we are running on the real robot
    {
        _is_sim = false;
    }

}

void CartImpCntrlRt::is_dummy(std::string dummy_string = "dummy")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t dummy_found = _hw_type.find(dummy_string);

    if (dummy_found != std::string::npos) { // we are running the plugin in dummy mode

        _is_dummy = true;
    }
    else // we are running on the real robot
    {
        _is_dummy = false;
    }

}

void CartImpCntrlRt::init_vars()
{

    _q_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _jnt_stiffness_setpoint = Eigen::VectorXd::Zero(_n_jnts_robot);
    _jnt_damping_setpoint = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_ff = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_model);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _effort_lims = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_model = Eigen::VectorXd::Zero(_n_jnts_model);
    _v_model = Eigen::VectorXd::Zero(_n_jnts_model);
    _v_dot_model = Eigen::VectorXd::Zero(_n_jnts_model);
    _tau_model = Eigen::VectorXd::Zero(_n_jnts_model);

    _Jc = Eigen::MatrixXd::Zero(6, _n_jnts_model);
    _B = Eigen::MatrixXd::Zero(6, _n_jnts_model);

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        _jnt_stiffness_setpoint(i) = _torque_cntrl_stiffness_setpoint;
        _jnt_damping_setpoint(i) = _torque_cntrl_damping_setpoint;

    }

    _K.setZero();
    _D.setZero();
    _Lambda_inv.setZero();

    _k.setZero();
    _d.setZero();
    _pos_ref << -0.3, 0.0, -0.3;
    _R_ref = Eigen::MatrixXd::Identity(3, 3);
    _v_ref.setZero();
    _a_ref.setZero();

}

void CartImpCntrlRt::get_params_from_config()
{

    _urdf_path = getParamOrThrow<std::string>("~urdf_path");

    _srdf_path = getParamOrThrow<std::string>("~srdf_path");

    _ci_yaml_path = getParamOrThrow<std::string>("~ci_yaml_path");

}

void CartImpCntrlRt::init_model_interface()
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

template <typename T>
std::shared_ptr<T> task_as(Cartesian::TaskDescription::Ptr t)
{
    return std::dynamic_pointer_cast<T>(t);
}

void CartImpCntrlRt::init_cartesio_solver()
{

    auto ci_params = std::make_shared<Cartesian::Parameters>(_plugin_dt);
    ci_params->setLogEnabled(true);

    auto ci_ctx = std::make_shared<Cartesian::Context>(ci_params,
                _model);

    // load problem
    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## CartImpCntrlRt: trying to load problem description for CartesIO @ {}\n \n", _ci_yaml_path);

    // Load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(_ci_yaml_path);

    Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ci_ctx);

    _ci_solver = Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                                 ik_pb,
                                                                 ci_ctx);

    _task_list = _ci_solver->getTaskList();

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## CartImpCntrlRt:\nTASK LIST:\n");
    for (std::string task: _task_list)
    {

        jhigh().jprint(fmt::fg(fmt::terminal_color::green),
            "\n- {}", task);
    }

    // getting tasks
//    _ground_contact = task_as<Cartesian::CartesianTask>(_ci_solver->getTask("ground_contact"));
//    _actuated_jnt_tracking = task_as<Cartesian::PosturalTask>(_ci_solver->getTask("actuated_jnt_tracking"));
//    _torque_limits = task_as<Cartesian::acceleration::TorqueLimits>(_ci_solver->getTask("effort_limits"));

    _cart_impedance_task = task_as<Cartesian::InteractionTask>(_ci_solver->getTask("cart_impedance"));
    _touchdown_conf = task_as<Cartesian::PosturalTask>(_ci_solver->getTask("touchdown_conf"));

    _cart_impedance = _cart_impedance_task->getImpedance();
    _K = _cart_impedance.stiffness;
    _D = _cart_impedance.damping;
    _k_setpoint = _K.diagonal();
    _d_setpoint = _D.diagonal();
    _cart_impedance_task->getPoseReference(_M_ref_imp);

}

void CartImpCntrlRt::update_ci_solver()
{

    _ci_solver->update(_loop_time, _plugin_dt);

}

void CartImpCntrlRt::compute_inverse_dyn()
{

    _model->computeInverseDynamics(_tau_cmd);

}

void CartImpCntrlRt::saturate_input()
{
    _robot->enforceJointLimits(_q_cmd);
    _robot->enforceVelocityLimit(_q_dot_cmd);
    _robot->enforceEffortLimit(_tau_ff);
}

void CartImpCntrlRt::init_logger()
{
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/LandingImpCntrlRt", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _logger->add("plugin_dt", _plugin_dt);
    _logger->add("is_sim", int(_is_sim));
    _logger->add("eff_limits", _effort_lims);

    _logger->create("Lambda_inv", 6, 6, _matlogger_buffer_size);
    _logger->create("Jc", 6, _n_jnts_model, _matlogger_buffer_size);
    _logger->create("B", _n_jnts_model, _n_jnts_model, _matlogger_buffer_size);
    _logger->create("k", 6, 1, _matlogger_buffer_size);
    _logger->create("d", 6, 1, _matlogger_buffer_size);
    _logger->create("k_setpoint", 6, 1, _matlogger_buffer_size);
    _logger->create("d_setpoint", 6, 1, _matlogger_buffer_size);
    _logger->create("p_ref", 3, 1, _matlogger_buffer_size);
    _logger->create("R_ref", 3, 3, _matlogger_buffer_size);
    _logger->create("v_ref", 6, 1, _matlogger_buffer_size);
    _logger->create("a_ref", 6, 1, _matlogger_buffer_size);

    _logger->create("q_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _logger->create("q_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _logger->create("q_model", _n_jnts_model, 1, _matlogger_buffer_size);
    _logger->create("v_model", _n_jnts_model, 1, _matlogger_buffer_size);
    _logger->create("v_dot_model", _n_jnts_model, 1, _matlogger_buffer_size);
    _logger->create("tau_model", _n_jnts_model, 1, _matlogger_buffer_size);
    _logger->create("jnt_stiffness_setpoint", _n_jnts_robot, 1, _matlogger_buffer_size);
    _logger->create("jnt_damping_setpoint", _n_jnts_robot, 1, _matlogger_buffer_size);
    _logger->create("tau_ff", _n_jnts_robot, 1, _matlogger_buffer_size);
    _logger->create("tau_cmd", _n_jnts_model, 1, _matlogger_buffer_size);
    _logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

    _logger->create("K_setpoint", 6, 6, _matlogger_buffer_size);
    _logger->create("D_setpoint", 6, 6, _matlogger_buffer_size);

}

void CartImpCntrlRt::add_data2logger()
{

    _logger->add("Lambda_inv", _Lambda_inv);
    _logger->add("Jc", _Jc);
    _logger->add("B", _B);
    _logger->add("k", _k);
    _logger->add("d", _d);
    _logger->add("k_setpoint", _k_setpoint);
    _logger->add("d_setpoint", _d_setpoint);
    _logger->add("R_ref", _R_ref);
    _logger->add("p_ref", _pos_ref);
    _logger->add("v_ref", _v_ref);
    _logger->add("a_ref", _a_ref);

    _logger->add("q_meas", _q_meas);
    _logger->add("q_dot_meas", _q_dot_meas);
    _logger->add("q_model", _q_model);
    _logger->add("v_model", _v_model);
    _logger->add("v_dot_model", _v_dot_model);
    _logger->add("tau_model", _tau_model);
    _logger->add("jnt_stiffness_setpoint", _jnt_stiffness_setpoint);
    _logger->add("jnt_damping_setpoint", _jnt_damping_setpoint);
    _logger->add("tau_ff", _tau_ff);
    _logger->add("tau_cmd", _tau_cmd);
    _logger->add("tau_meas", _tau_meas);

    _logger->add("K_setpoint", _K_setpoint);
    _logger->add("D_setpoint", _D_setpoint);
}

bool CartImpCntrlRt::on_initialize()
{
    is_sim();

    is_dummy();

    // Getting nominal control period from plugin method
    _plugin_dt = getPeriodSec();

    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    // Initializing XBot2 model interface using the read parameters 
    init_model_interface();

    // Setting robot control mode, stiffness and damping
    _n_jnts_robot = _robot->getJointNum();

    // Reading joint effort limits (used for saturating the trajectory)
    _model->getEffortLimits(_effort_lims);

    init_vars();

    return true;
}

void CartImpCntrlRt::starting()
{
    init_clocks();

    init_logger();

    // Initializing CartesIO solver (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the solver is recreated)

    init_cartesio_solver();

    // Reset CartesIO solver
    _ci_solver->reset(_loop_time);

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() +
                           ControlMode::Stiffness() + ControlMode::Damping());

    // Move on to run()
    start_completed();
}

void CartImpCntrlRt::update_tasks()
{
    _Lambda_inv.noalias() = _Jc * _B.inverse() * _Jc.transpose(); // inverse of cartesian inertia matrix

//    _K_setpoint.noalias() = _Lambda_inv * _k_setpoint.asDiagonal();
//    _D_setpoint.noalias() = _Lambda_inv * _d_setpoint.asDiagonal();

    _K_setpoint = _k_setpoint.asDiagonal();
    _D_setpoint = _d_setpoint.asDiagonal();

    _cart_impedance_setpoint.stiffness = _K_setpoint;
    _cart_impedance_setpoint.damping = _D_setpoint;

    _cart_impedance_task->setImpedance(_cart_impedance_setpoint);

    _M_ref_imp.translation() = _pos_ref;
    _M_ref_imp.linear() = _R_ref;
    _v_ref.setZero();
    _a_ref.setZero();
    _cart_impedance_task->setPoseReference(_M_ref_imp);
    _cart_impedance_task->setVelocityReference(_v_ref);
    _cart_impedance_task->setAccelerationReference(_a_ref);

     // _touchdown_conf->setActivationState(Cartesian::ActivationState::Enabled);
    // _touchdown_conf->setActivationState(Cartesian::ActivationState::Disabled);

}

void CartImpCntrlRt::update_state()
{
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_meas);
    _robot->getMotorVelocity(_q_dot_meas);

    // Updating the model with the measurements
    _q_model.segment(1, _n_jnts_robot) = _q_meas;
    _v_model.segment(1, _n_jnts_robot) = _q_dot_meas;
    _model->setJointPosition(_q_model);
    _model->setJointVelocity(_v_model);
    _model->update();
    _model->getRelativeJacobian(_cart_impedance_task->getDistalLink(),
                        _cart_impedance_task->getBaseLink(),
                        _Jc);
    _model->getInertiaMatrix(_B);

    update_tasks();

    update_ci_solver();

    _model->getJointEffort(_tau_cmd);
//    _model->setJointPosition(_q_model);
//    _model->setJointVelocity(_v_model);
//    _model->update();

//    compute_inverse_dyn();

}

void CartImpCntrlRt::run()
{   

    // Update the measured state
    update_state();

    // Set the effort commands (and also stiffness/damping)
    _tau_ff = _tau_cmd.segment(1, _n_jnts_robot);
    _q_cmd = _q_meas;
    _q_dot_cmd = _q_dot_meas;
    // Check input for bound violations
    saturate_input();

    _robot->setEffortReference(_tau_ff);

    _robot->setPositionReference(_q_cmd); // this way the error is 0
    _robot->setVelocityReference(_q_dot_cmd); // this way the error is 0
    _robot->setStiffness(_jnt_stiffness_setpoint);
    _robot->setDamping(_jnt_damping_setpoint);
    _robot->move();

    // Getting cartesian imp. for debugging purposes
    _cart_impedance= _cart_impedance_task->getImpedance();
    _K = _cart_impedance.stiffness;
    _D = _cart_impedance.damping;
    _k = _K.diagonal();
    _d = _D.diagonal();

    add_data2logger();

    update_clocks(); // updating clocks

}

void CartImpCntrlRt::on_stop()
{

    // Read the current state
    _robot->sense();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();

    // Destroy CartesIO solver (will be recreated upon plugin restart)
    _ci_solver.reset();
    
}

XBOT2_REGISTER_PLUGIN(CartImpCntrlRt, cart_imp_cntrl_rt)
