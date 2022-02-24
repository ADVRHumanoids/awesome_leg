#include "cartesio_imp_cntrl_ros_rt.h"

void CartesioImpCntrlRosRt::get_params_from_config()
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

void CartesioImpCntrlRosRt::init_model_interface()
{
    
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path);
    xbot_cfg.set_srdf_path(_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // Initializing XBot2 ModelInterface for the rt thread
    _model = XBot::ModelInterface::getModel(xbot_cfg); 
    _n_jnts_model = _model->getJointNum();

    // Initializing XBot2 ModelInterface for the nrt thread
    _nrt_model = ModelInterface::getModel(xbot_cfg);
}

void CartesioImpCntrlRosRt::init_cartesio_solver()
{

    // Building context for rt thread

    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(_dt),
                _model);

    // Load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(_cartesio_path);

    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // We are finally ready to make the CartesIO solver "OpenSot"
    _solver = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_pb, ctx);

    // Building context for nrt thread

    auto nrt_ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(*_solver->getContext()->params()),
                _nrt_model);

    _nrt_solver = std::make_shared<LockfreeBufferImpl>(_solver.get(), nrt_ctx);
    _nrt_solver->pushState(_solver.get(), _model.get());
    _nrt_solver->updateState();

}

void CartesioImpCntrlRosRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian");
    _ros_srv = std::make_shared<RosServerClass>(_nrt_solver, opt);
}

void CartesioImpCntrlRosRt::spawn_rnt_thread()
{

    /* Initialization */
    _rt_active = false;

    _nrt_exit = false;
    
    /* Spawn thread */
    _nrt_thread = std::make_unique<thread>(
        [this]()
        {
            this_thread::set_name("ci_imp_ros_nrt");

            while(!this->_nrt_exit)
            {
                this_thread::sleep_for(10ms);

                if(!this->_rt_active) continue;

                this->_nrt_solver->updateState();
                this->_ros_srv->run();

            }

        });
}

void CartesioImpCntrlRosRt::update_state()
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

void CartesioImpCntrlRosRt:: compute_joint_efforts()
{
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas); 
    _model->update();
    _model->getJointAcceleration(_q_p_ddot_ci); 
    _model->computeInverseDynamics(_effort_command);
}

bool CartesioImpCntrlRosRt::on_initialize()
{

    _nh = std::make_unique<ros::NodeHandle>();
    
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
    _n_jnts_robot = _robot->getJointNum();

    // Initializing CartesIO solver, ros server and spawning the non rt thread
    init_cartesio_solver();
    create_ros_api();
    spawn_rnt_thread();

    return true;
}

void CartesioImpCntrlRosRt::starting()
{
    // Creating a logger for post-processing (inserted here and not in on_initialize() 
    // so that when the plugin is restarted, the object is recreated)

    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/CartesioImpCntrlRosRt", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);


    auto task = _solver->getTask("tip"); // Getting tip cartesian task and casting it to cartesian (task)
    _int_task = std::dynamic_pointer_cast<InteractionTask>(task); // interaction cartesian task (exposes additional methods)
    _cart_task = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_cart_task)
    {
        jerror("tip task not cartesian");
    }

    _model->setJointPosition(_q_p_target);
    _model->update();
    _model->getPose("tip", _target_pose); 

    // initializing time (used for interpolation of trajectories inside CartesIO)
    _time = 0.0;

    // // Update the model with the current robot state
    update_state();  

    // Reset CartesIO solver
    _solver->reset(_time);

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // signal nrt thread that rt is active
    _rt_active = true;

    // Move on to run()
    start_completed();
}

void CartesioImpCntrlRosRt::run()
{   
    /* Receive commands from nrt */
    _nrt_solver->callAvailable(_solver.get());
    
    /* Send state to nrt */
    _nrt_solver->pushState(_solver.get(), _model.get());

    // Update the measured state
    update_state();
     
    // and update CartesIO solver using the measured state
    _solver->update(_time, _dt);

    // Read the joint efforts computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _robot->getJointEffort(_meas_effort);
    
    // Set the effort commands (and also stiffness/damping)
    _robot->setPositionReference(_q_p_meas); // sending also position reference only to improve the transition between torque and position control when stopping the plugin
    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time
    _time += _dt;

    // Logging stuff

    _logger->add("meas_efforts", _meas_effort);
    _logger->add("computed_efforts", _effort_command);

    if (_int_task)
    {
        _impedance= _int_task->getImpedance();
        _cart_stiffness = _impedance.stiffness; 
        _cart_damping = _impedance.damping;
        _logger->add("cartesian_stiffness", _cart_stiffness);
        _logger->add("cartesian_damping", _cart_damping);
    }

}

void CartesioImpCntrlRosRt::on_stop()
{
    // Read the current state
    update_state();
    // Setting references before exiting
    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());
    _robot->setPositionReference(_q_p_meas);
    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();
}

void CartesioImpCntrlRosRt::stopping()
{
    _rt_active = false;
    stop_completed();
}

XBOT2_REGISTER_PLUGIN(CartesioImpCntrlRosRt, cartesio_imp_cntrl_ros_rt)