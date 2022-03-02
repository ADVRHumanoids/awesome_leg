#include "grav_comp_cart_rt.h"

void GravCompCartesio::get_params_from_config()
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
}

void GravCompCartesio::init_model_interface()
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

void GravCompCartesio::init_cartesio_solver()
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

void GravCompCartesio::update_state()
{
    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);    

    // Updating the model with the measurements
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas);
    _model->update();

    // Reset CartesIO solver to use an updated measured tip position
    _solver->reset(_time);

    _solver->update(_time, _dt);
    
}

void GravCompCartesio::saturate_input()
{
    int input_sign = 1; // defaults to positive sign 

    for(int i = 0; i < _n_jnts_model; i++)
    {
        if (abs(_effort_command[i]) >= abs(_effort_lims[i]))
        {
            input_sign = (signbit(_effort_command[i])) ? -1: 1; 

            _effort_command[i] = input_sign * (abs(_effort_lims[i]) - _delta_effort_lim);
        }
    }
}

bool GravCompCartesio::on_initialize()
{
    // Getting nominal control period from plugin method
    _dt = getPeriodSec();

    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    // Initializing XBot2 model interface using the read parameters 
    init_model_interface();

    // Reading joint effort limits (used for saturating the trajectory)
    _model->getEffortLimits(_effort_lims);
    
    return true;
}

void GravCompCartesio::starting()
{
    // Creating a logger for post-processing (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the object is recreated)

    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/GravCompRt", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Initializing CartesIO solver
    init_cartesio_solver();

    // initializing time (used for interpolation of trajectories inside CartesIO)
    _time = 0;

    // "sensing" the robot
    _robot->sense();

    // Update the model with the current robot state
    _model->update(); 

    // Reset CartesIO solver
    _solver->reset(_time);

    // Setting robot control mode, stiffness and damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping()); 

    // Move on to run()
    start_completed();
}

void GravCompCartesio::run()
{   
    // Update the measured state and update CartesIO solver
    update_state();

    // Read the joint effort computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _effort_command = _effort_command + _tau_tilde; // adding torque compensation drift 
    
    // Check input for bound violations
    saturate_input();

    // Set the effort commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time
    _time += _dt;

    _logger->add("computed_efforts", _effort_command);
    _logger->add("q_p", _q_p_meas);
}

void GravCompCartesio::on_stop()
{
    // Setting references before exiting
    
    _robot->sense();

    _robot->getJointPosition(_q_p_meas);
    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    _robot->setPositionReference(_q_p_meas);
    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();

    // Destroy CartesIO solver (will be recreated upon plugin restart)
    _solver.reset();
    
}

XBOT2_REGISTER_PLUGIN(GravCompCartesio, grav_comp_cart_rt)