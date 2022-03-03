#include "cartesio_elliptical_rt.h"

void CartesioEllipticalRt::get_params_from_config()
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
    
    bool t_exec_traj_found = getParam("~t_exec_traj", _t_exec_traj);
    bool n_samples_found = getParam("~n_samples", _n_samples);

    bool delta_effort_lim_found = getParam("~delta_effort_lim", _delta_effort_lim);

    bool use_vel_ff_found = getParam("~use_vel_ff", _use_vel_ff);
    bool use_acc_ff_found = getParam("~use_acc_ff", _use_acc_ff);

    bool a_found = getParam("~a", _a_ellps);
    bool b_found = getParam("~b", _b_ellps);
    bool x_c_found = getParam("~x_c", _x_c_ellps);
    bool z_c_found = getParam("~z_c", _z_c_ellps);
    bool alpha_found = getParam("~alpha", _alpha);

    bool interaction_found = getParam("~is_interaction", _is_interaction);

}

void CartesioEllipticalRt::init_model_interface()
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

void CartesioEllipticalRt::init_cartesio_solver()
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

    // Building context for nrt thread (used to expose the ROS interface)

    auto nrt_ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(*_solver->getContext()->params()),
                _nrt_model);

    _nrt_solver = std::make_shared<LockfreeBufferImpl>(_solver.get(), nrt_ctx);
    _nrt_solver->pushState(_solver.get(), _model.get());
    _nrt_solver->updateState();

}

void CartesioEllipticalRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci_elliptical");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian_elliptical");
    _ros_srv = std::make_shared<RosServerClass>(_nrt_solver, opt);
}

void CartesioEllipticalRt::spawn_rnt_thread()
{

    /* Initialization */
    _rt_active = false;

    _nrt_exit = false;
    
    /* Spawn thread */
    _nrt_thread = std::make_unique<thread>(
        [this]()
        {
            this_thread::set_name("ci_ellipt_nrt");

            while(!this->_nrt_exit)
            {
                this_thread::sleep_for(10ms);

                if(!this->_rt_active) continue;

                this->_nrt_solver->updateState();
                this->_ros_srv->run();

            }

        });
}

void CartesioEllipticalRt::update_state()
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

void CartesioEllipticalRt::saturate_input()
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

void CartesioEllipticalRt::compute_ref_traj(double time)
{
    Eigen::Vector3d traj_offset, traj, traj_dot, traj_ddot, traj_wrt_center, rot_axis;
    rot_axis << 0, 1, 0; // rotate trajectory around y axis (same direction as the hip joint)


    // Traj wrt rotated frame

    traj_offset << _x_c_ellps, 
                   0, 
                   _z_c_ellps;

    traj_wrt_center << _a_ellps * cos(2 * M_PI * time/_t_exec_traj),
            0,
            _b_ellps * sin(2 * M_PI * time/_t_exec_traj);

    traj = traj_offset + traj_wrt_center;

    // Target velocity
    traj_dot << - _a_ellps * 2 * M_PI / _t_exec_traj * sin(2 * M_PI * time/_t_exec_traj), 
                0, 
                _b_ellps * 2 * M_PI / _t_exec_traj * cos(2 * M_PI * time/_t_exec_traj);

    // Target acceleration
    traj_ddot << - _a_ellps * 4 * pow(M_PI, 2) / pow(_t_exec_traj, 2) * cos(2 * M_PI * time/_t_exec_traj), 
                0, 
                -_b_ellps * 4 * pow(M_PI, 2) / pow(_t_exec_traj, 2) * sin(2 * M_PI * time/_t_exec_traj);

    
    // Rotating everything in the base frame

    Eigen::Affine3d A_pos = Eigen::Translation3d(traj_offset) * Eigen::AngleAxisd(- _alpha, rot_axis) * Eigen::Translation3d(-traj_offset); // affine transf. for the position trajectory 
    Eigen::AngleAxisd A_dot = Eigen::AngleAxisd(- _alpha, rot_axis); // affine transf. for the vel and acceleration trajectories

    traj = A_pos * traj; 
    traj_dot = A_dot * traj_dot; // note: the rotated reference frame is stationary wrt the base frame
    traj_ddot = A_dot * traj_ddot;

    _target_pose.translation() = traj; // set the translational component of the target pose

    _target_vel << traj_dot, 
                   Eigen:: ArrayXd::Zero(3);

    _target_acc << traj_ddot, 
                   Eigen:: ArrayXd::Zero(3);

}

bool CartesioEllipticalRt::on_initialize()
{
    _nh = std::make_unique<ros::NodeHandle>();
    
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

    // Reading joint effort limits (used for saturating the trajectory)
    _model->getEffortLimits(_effort_lims);

    return true;
}

void CartesioEllipticalRt::starting()
{

    // Creating a logger for post-processing
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/CartesioEllipticalRt_log", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Getting tip cartesian task and casting it to cartesian (task)
    auto task = _solver->getTask("tip");

    _int_task = std::dynamic_pointer_cast<InteractionTask>(task);
    _cart_task = std::dynamic_pointer_cast<CartesianTask>(task);
    
    if(!_cart_task)
    {
        jerror("tip task not cartesian");
    }

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

void CartesioEllipticalRt::run()
{   
    /* Receive commands from nrt */
    _nrt_solver->callAvailable(_solver.get());
    
    /* Send state to nrt */
    _nrt_solver->pushState(_solver.get(), _model.get());

    // Update the measured state
    update_state();
    
    // Update (_target_pose) and set tip pose target 
    compute_ref_traj(_time);

    // Setting target trajectory 
    _cart_task->setPoseReference(_target_pose); 

    if (_use_vel_ff)
    {
        _cart_task->setVelocityReference(_target_vel);
    }

    if (_use_acc_ff)
    {
         _cart_task->setAccelerationReference(_target_acc); 
    }

    // jwarn("_target_pose:\n");
    // for(int i=0; i<(_target_pose.translation()).rows();i++)  // loop 3 times for three lines
    //   {
    //     for(int j=0;j<(_target_pose.translation()).cols();j++)  // loop for the three elements on the line
    //     {
    //         jwarn("{}", _target_pose.translation()(i, j));  // display the current element out of the array
    //         jwarn("\t");
    //     }
    //   }
    // jwarn("\n");

    if (_int_task)
    {
   
        // Logging impedance for post-processing
        _impedance= _int_task->getImpedance();
        _cart_stiffness = _impedance.stiffness; 
        _cart_damping = _impedance.damping;
        _logger->add("cartesian_stiffness", _cart_stiffness);
        _logger->add("cartesian_damping", _cart_damping);
    }

    // and update CartesIO solver using the measured state
    _solver->update(_time, _dt);

    // Read the joint efforts computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _robot->getJointEffort(_meas_effort);

    // Check input for bound violations
    saturate_input();     

    // Set the effort commands (and also stiffness/damping)
    _robot->setPositionReference(_q_p_meas); // sending also position reference only to improve the transition between torque and position control when stopping the plugin
    _robot->setEffortReference(_effort_command);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time
    _time += _dt;

    if (_time >= _t_exec_traj)
    {
        _time = _time - _t_exec_traj;
    }
    
    // Getting tip pose for debugging
    _model->getPose("tip", _meas_pose);

    _logger->add("meas_efforts", _meas_effort);
    _logger->add("computed_efforts", _effort_command);
    
    _logger->add("target_tip_pos", _target_pose.translation());
    _logger->add("meas_tip_pos", _meas_pose.translation());

    if (_int_task)
    {
        _impedance= _int_task->getImpedance();
        _cart_stiffness = _impedance.stiffness; 
        _cart_damping = _impedance.damping;
        _logger->add("cartesian_stiffness", _cart_stiffness);
        _logger->add("cartesian_damping", _cart_damping);
    }

}

void CartesioEllipticalRt::on_stop()
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

void CartesioEllipticalRt::stopping()
{
    _rt_active = false;
    stop_completed();
}

void CartesioEllipticalRt::on_abort()
{
    _rt_active = false;
    _nrt_exit = true;
}

void CartesioEllipticalRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_thread) _nrt_thread->join();
}

XBOT2_REGISTER_PLUGIN(CartesioEllipticalRt, cartesio_elliptical_rt)