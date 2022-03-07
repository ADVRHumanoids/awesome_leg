#include "cartesio_ell_rt.h"

bool CartesioEllRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    bool tau_tilde_found = getParam("~torque_bias", _tau_tilde); // estimated bias torques
    bool urdf_path_found = getParam("~urdf_path", _urdf_path); // urdf specific to gravity compensator
    bool srdf_path_found = getParam("~srdf_path", _srdf_path); // srdf_path specific to gravity compensator
    bool cartesio_path_found = getParam("~cartesio_yaml_path", _cartesio_path); // srdf_path specific to gravity compensator
    bool stiffness_found = getParam("~stiffness", _stiffness);
    bool damping_found = getParam("~damping", _damping);
    bool stop_stiffness_found = getParam("~stop_stiffness", _stop_stiffness);
    bool stop_damping_found = getParam("~stop_damping", _stop_damping);

    bool delta_effort_lim_found = getParam("~delta_effort_lim", _delta_effort_lim);

    bool use_vel_ff_found = getParam("~use_vel_ff", _use_vel_ff);
    bool use_acc_ff_found = getParam("~use_acc_ff", _use_acc_ff);

    bool traj_prm_rmp_time_found = getParam("~traj_prm_rmp_time", _traj_prm_rmp_time);
    bool t_exec_traj_found = getParam("~t_exec_traj", _t_exec_traj_trgt);
    bool t_exec_lb_found = getParam("~t_exec_lb", _t_exec_lb);
    bool forward_found = getParam("~is_forward", _is_forward);
    bool a_found = getParam("~a", _a_ellps_trgt);
    bool b_found = getParam("~b", _b_ellps_trgt);
    bool x_c_found = getParam("~x_c", _x_c_ellps_trgt);
    bool z_c_found = getParam("~z_c", _z_c_ellps_trgt);
    bool alpha_found = getParam("~alpha", _alpha_trgt);

    if (
        !(tau_tilde_found && 
        urdf_path_found && srdf_path_found && cartesio_path_found &&
        stiffness_found && damping_found && stop_stiffness_found && stop_damping_found &&
        delta_effort_lim_found && 
        use_vel_ff_found && use_acc_ff_found &&
        traj_prm_rmp_time_found && t_exec_traj_found && t_exec_lb_found && forward_found &&
        a_found && b_found && x_c_found && z_c_found && alpha_found)
        )
    { // not all necessary parameters were read -> throw error
        jhigh().jerror("Failed to read at least one of the plugin parameters from the YAML file.\n Please check that you have assigned all of them correctly.");
                
        return false;
    }

    if (abs(_t_exec_traj) < abs(_t_exec_lb))
    { // saturating (towards its lower bound) the trajectory execution time for safety reasons.
        _t_exec_traj = _t_exec_lb; 
        jhigh().jwarn("The selected t_exec_traj is less than the set t_exec_lb.\n Setting t_exec_traj to {} s.", _t_exec_lb);  
    }

    return true;
}

void CartesioEllRt::init_model_interface()
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

void CartesioEllRt::init_cartesio_solver()
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

void CartesioEllRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci_ell");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian_ell");
    _ros_srv = std::make_shared<RosServerClass>(_nrt_solver, opt);
}

void CartesioEllRt::spawn_rnt_thread()
{

    /* Initialization */
    _rt_active = false;

    _nrt_exit = false;
    
    /* Spawn thread */
    _nrt_thread = std::make_unique<thread>(
        [this]()
        {
            this_thread::set_name("ci_ell_nrt");

            while(!this->_nrt_exit)
            {
                this_thread::sleep_for(10ms);

                if(!this->_rt_active) continue;

                this->_nrt_solver->updateState();
                this->_ros_srv->run();

            }

        });
}

void CartesioEllRt::saturate_input()
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

void CartesioEllRt::update_state()
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

void CartesioEllRt::compute_ref_traj(double time)
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
                   Eigen::ArrayXd::Zero(3);

    _target_acc << traj_ddot, 
                   Eigen::ArrayXd::Zero(3);

}

void CartesioEllRt::on_ell_traj_recv(const awesome_leg_pholus::EllTrajRt& msg)
{
    _traj_par_callback_trigger = true; // to signal to other methods that a callback was received
    
    // Resetting parameters trajectory time 
    _time_traj_par = 0;

    _is_forward = msg.is_forward; // trajectory direction

    // Assigning read trajectory parameters to target variables
    _t_exec_traj_trgt = abs(msg.t_exec);
    _x_c_ellps_trgt = msg.x_c;
    _z_c_ellps_trgt = msg.z_c;
    _a_ellps_trgt = _is_forward ? abs(msg.a_ellps): - abs(msg.a_ellps); 
    _b_ellps_trgt = abs(msg.b_ellps);
    _alpha_trgt = msg.alpha;

    _use_vel_ff = msg.use_vel_ff;
    _use_acc_ff = msg.use_acc_ff;

    // Assigning current trajectory parameters values
    _t_exec_traj_init = _t_exec_traj;
    _x_c_ellps_init = _x_c_ellps;
    _z_c_ellps_init = _z_c_ellps;
    _a_ellps_init = _a_ellps;
    _b_ellps_init = _b_ellps;
    _alpha_init = _alpha;
    
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n Received new traj. configuration: \n");

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "t_exec: {}\n", _t_exec_traj_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "a_ellps: {}\n", _a_ellps_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "b_ellps: {}\n", _b_ellps_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "x_c_ellps: {}\n", _x_c_ellps_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "z_c_ellps: {}\n", _z_c_ellps_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "alpha: {}\n", _alpha_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "use velocity feedforward: {}\n", _use_vel_ff);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "use acceleration feedforward: {}\n", _use_acc_ff);

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta), "\n");

}

void CartesioEllRt::peisekah_transition()
{
    if (_traj_par_callback_trigger) // a callback was received (defaults to false)
    {
        if (_time_traj_par >= _traj_prm_rmp_time) // parameter transition trajectory completed --> set params to their target, to avoid numerical errors
        {

            _t_exec_traj = _t_exec_traj_trgt;
            _x_c_ellps = _x_c_ellps_trgt;
            _z_c_ellps = _z_c_ellps_trgt;
            _a_ellps = _a_ellps_trgt;
            _b_ellps = _b_ellps_trgt;
            _alpha = _alpha_trgt;

            _traj_par_callback_trigger = false; // reset callback counter, since parameters' trajectory has terminated
        }
        else
        {
            // Smooth transition for the traj paramters changed via ROS topic 
            double common_part_traj = (126.0 * pow(_time_traj_par/_traj_prm_rmp_time, 5) - 420.0 * pow(_time_traj_par/_traj_prm_rmp_time, 6) + 540.0 * pow(_time_traj_par/_traj_prm_rmp_time, 7) - 315.0 * pow(_time_traj_par/_traj_prm_rmp_time, 8) + 70.0 * pow(_time_traj_par/_traj_prm_rmp_time, 9));
            
            _t_exec_traj = _t_exec_traj_init + (_t_exec_traj_trgt - _t_exec_traj_init) *  common_part_traj;
            _x_c_ellps = _x_c_ellps_init + (_x_c_ellps_trgt - _x_c_ellps_init) *  common_part_traj;
            _z_c_ellps = _z_c_ellps_init + (_z_c_ellps_trgt - _z_c_ellps_init) *  common_part_traj;
            _a_ellps = _a_ellps_init + (_a_ellps_trgt - _a_ellps_init) *  common_part_traj;
            _b_ellps = _b_ellps_init + (_b_ellps_trgt - _b_ellps_init) *  common_part_traj;
            _alpha = _alpha_init + (_alpha_trgt - _alpha_init) *  common_part_traj;
        }

        _time_traj_par += _dt; // incrementing trajory parameter time
    }

}

bool CartesioEllRt::on_initialize()
{

    // create a nodehandle with namespace equal to
    // the plugin name
    ros::NodeHandle nh(getName());
    // use it to create our 'RosSupport' class
    // note: this must run *after* ros::init, which
    // at this point is guaranteed to be true
    _ros = std::make_unique<RosSupport>(nh);

    /* Subscriber */
    _ell_traj_sub = _ros->subscribe("my_ell_traj",
                                &CartesioEllRt::on_ell_traj_recv,
                                this,
                                1,  // queue size
                                &_queue);
    
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

void CartesioEllRt::starting()
{
    // Resetting flag for reaching the initial tip position
    _first_run = true;

    // Creating a logger for post-processing
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/CartesioEllConfigRt_log", opt); // date-time automatically appended
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
    // initializing time (used for interpolation of trajectory parameters)
    _time_traj_par = 0.0;

    // Update the model with the current robot state
    update_state();  

    // Reset CartesIO solver
    _solver->reset(_time);

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // signal nrt thread that rt is active
    _rt_active = true;

    _logger->add("plugin_dt", _dt);
    _logger->add("stop_stiffness", _stop_stiffness);
    _logger->add("stop_damping", _stop_damping);
    _logger->add("stiffness", _stiffness);
    _logger->add("damping", _damping);
    _logger->add("use_vel_ff", _use_vel_ff);
    _logger->add("use_acc_ff", _use_acc_ff);
    _logger->add("t_exec_lb", _t_exec_lb);
    _logger->add("traj_prm_rmp_time",_traj_prm_rmp_time);

    // Move on to run()
    start_completed();

}

void CartesioEllRt::run()
{   
    if (_first_run)
    { // first time entering the run

        _traj_par_callback_trigger = true; //  faking a received trajectory configuration

        update_state();
        _model->getPose("tip", _meas_pose); // getting initial tip position
        
        _t_exec_traj_init = _t_exec_traj_trgt;
        _a_ellps_init = 0;
        _b_ellps_init = 0;
        _alpha_init = 0;
        _x_c_ellps_init = _meas_pose.translation()[0];
        _z_c_ellps_init = _meas_pose.translation()[2];

        _first_run = false;
        
    }
    // process callbacks
    _queue.run();
    
    // Compute a smooth transition for the (potentially) changed trajectory parameters. 
    peisekah_transition();
    
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

    // and update CartesIO solver using the measured state
    _solver->update(_time, _dt);

    // Read the joint efforts computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _robot->getJointEffort(_meas_effort);
    
    // Check input for bound violations
    saturate_input(); 

    // Set the commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command + _tau_tilde);
    _robot->setPositionReference(_q_p_meas);
    _robot->setStiffness(_stiffness);
    _robot->setDamping(_damping);

    // Send commands to the robot
    _robot->move(); 

    // Update time(s)
    _time += _dt;
    
    if (_time >= _t_exec_traj)
    {
        _time = _time - _t_exec_traj;
    }
    
    // Getting some additional useful data
    _model->getPose("tip", _meas_pose);
    _model->getInertiaMatrix(_M);
    _model->getJacobian("tip", _J);

    // Adding that useful data to logger
    _logger->add("meas_efforts", _meas_effort);
    _logger->add("computed_efforts", _effort_command);

    _logger->add("M", _M);
    _logger->add("J", _J);
    
    _logger->add("tip_pos_ref", _target_pose.translation());
    _logger->add("tip_pos_meas", _meas_pose.translation());

    _logger->add("t_exec_traj", _t_exec_traj);
    _logger->add("a_ellps", _a_ellps);
    _logger->add("b_ellps", _b_ellps);
    _logger->add("x_c_ellps", _x_c_ellps);
    _logger->add("z_c_ellps", _z_c_ellps);
    _logger->add("alpha", _alpha);

    _logger->add("q_p_meas", _q_p_meas);
    _logger->add("q_p_dot_meas", _q_p_dot_meas);

    _logger->add("use_vel_ff", _use_vel_ff ? 1: 0);
    _logger->add("use_acc_ff", _use_acc_ff ? 1: 0);

    _logger->add("is_forward", _is_forward ? 1: 0);

    if (_int_task)
    {
        _impedance= _int_task->getImpedance();
        _cart_stiffness = _impedance.stiffness; 
        _cart_damping = _impedance.damping;
        _logger->add("cartesian_stiffness", _cart_stiffness);
        _logger->add("cartesian_damping", _cart_damping);
    }

}

void CartesioEllRt::on_stop()
{
    // Read the current state
    _robot->sense();

    // Setting references before exiting
    _robot->setControlMode(ControlMode::Position() + ControlMode::Stiffness() + ControlMode::Damping());

    _robot->setStiffness(_stop_stiffness);
    _robot->setDamping(_stop_damping);
    _robot->getPositionReference(_q_p_meas); // to avoid jumps in the references when stopping the plugin
    _robot->setPositionReference(_q_p_meas);

    // Sending references
    _robot->move();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _logger.reset();
}

void CartesioEllRt::stopping()
{
    _rt_active = false;
    stop_completed();
}

void CartesioEllRt::on_abort()
{
    _rt_active = false;
    _nrt_exit = true;
}

void CartesioEllRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_thread) _nrt_thread->join();
}

XBOT2_REGISTER_PLUGIN(CartesioEllRt, cartesio_ell_rt)