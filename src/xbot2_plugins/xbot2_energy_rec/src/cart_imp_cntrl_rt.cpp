#include "cart_imp_cntrl_rt.h"

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

    // getting tasks
    _ground_contact = task_as<Cartesian::CartesianTask>(_ci_solver->getTask("ground_contact"));
//    _hip_impedance = task_as<Cartesian::InteractionTask>(_ci_solver->getTask("hip_impedance"));
//    _actuated_jnt_tracking = task_as<Cartesian::PosturalTask>(_ci_solver->getTask("actuated_jnt_tracking"));
//    _touchdown_conf = task_as<Cartesian::PosturalTask>(_ci_solver->getTask("touchdown_conf"));
//    _torque_limits = task_as<Cartesian::acceleration::TorqueLimits>(_ci_solver->getTask("effort_limits"));

//    _int_task = std::dynamic_pointer_cast<InteractionTask>(task); // interaction cartesian task (exposes additional methods)

}

void CartImpCntrlRt::update_state()
{
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);    
    
    // Updating the model with the measurements
//    _model->setJointPosition(_q_p_meas);
//    _model->setJointVelocity(_q_p_dot_meas);
//    _model->update();
    
}

//void CartImpCntrlRt::saturate_input()
//{
//    int input_sign = 1; // defaults to positive sign

//    for(int i = 0; i < _n_jnts_model; i++)
//    {
//        if (abs(_effort_command[i]) >= abs(_effort_lims[i]))
//        {
//            input_sign = (signbit(_effort_command[i])) ? -1: 1;

//            _effort_command[i] = input_sign * (abs(_effort_lims[i]) - _delta_effort_lim);
//        }
//    }
//}

bool CartImpCntrlRt::on_initialize()
{
    
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
    
    return true;
}

void CartImpCntrlRt::starting()
{
    init_clocks();

    // Creating a logger for post-processing (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the object is recreated)

    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/LandingImpCntrlRt", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Initializing CartesIO solver (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the solver is recreated)

    init_cartesio_solver();

    // load opt imp trajectory and touchdown pose --> set reference hip position and flight cartesian reference
    // command reaching motion
//    _int_task->setPoseTarget(_target_pose, _t_exec);

    // // Update the model with the current robot state
    update_state();  

    // Reset CartesIO solver
    _ci_solver->reset(_loop_time);

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // Move on to run()
    start_completed();
}

void CartImpCntrlRt::run()
{   

    // Update the measured state
    update_state();
     
    // and update CartesIO solver using the measured state
//    _solver->update(_loop_time, _plugin_dt);

    // Read the joint efforts computed via CartesIO (computed using acceleration_support)
//    _model->getJointEffort(_effort_command);
//    _robot->getJointEffort(_meas_effort);
    
    // Check input for bound violations
//    saturate_input();

    // Set the effort commands (and also stiffness/damping)
//    _robot->setEffortReference(_effort_command);
//    _robot->setPositionReference(_q_p_meas); // sending also position reference only to avoid bad behavior when stopping the plugin
//    _robot->setStiffness(_stiffness);
//    _robot->setDamping(_damping);

    // Send commands to the robot
//    _robot->move();

    // Getting cartesian damping and stiffness for debugging purposes
//    _impedance= _int_task->getImpedance();
//    _cart_stiffness = _impedance.stiffness;
//    _cart_damping = _impedance.damping;

    update_clocks(); // updating clocks


}

void CartImpCntrlRt::on_stop()
{

    // Read the current state
    _robot->sense();

    // Setting references before exiting
//    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

//    _robot->setStiffness(_stop_stiffness);
//    _robot->setDamping(_stop_damping);
//    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
//    _robot->setPositionReference(_q_p_meas);

//    // Sending references
//    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();

    // Destroy CartesIO solver (will be recreated upon plugin restart)
    _ci_solver.reset();
    
}

XBOT2_REGISTER_PLUGIN(CartImpCntrlRt, cart_imp_cntrl_rt)
