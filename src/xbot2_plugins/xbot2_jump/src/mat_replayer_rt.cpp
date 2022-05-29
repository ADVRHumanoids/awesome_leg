#include "mat_replayer_rt.h"

bool MatReplayerRt::get_params_from_config()
{
    return true;
}

void MatReplayerRt::init_model_interface()
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

void MatReplayerRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "mat_replayer_rt");
    _ros_srv = std::make_shared<RosServerClass>(_nrt_solver, opt);
}

void MatReplayerRt::spawn_rnt_thread()
{

    /* Initialization */
    _rt_active = false;

    _nrt_exit = false;
    
    /* Spawn thread */
    _nrt_thread = std::make_unique<thread>(
        [this]()
        {
            this_thread::set_name("mat_replayer_nrt");

            while(!this->_nrt_exit)
            {
                this_thread::sleep_for(10ms);

                if(!this->_rt_active) continue;

                this->_nrt_solver->updateState();
                this->_ros_srv->run();

            }

        });
}

void MatReplayerRt::saturate_input()
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

void MatReplayerRt::update_state()
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

void MatReplayerRt::read_traj_sample(double time)
{

}

// bool MatReplayerRt::on_ell_traj_recv_srv(const awesome_leg_pholus::EllTrajRtRequest& req,
//                           awesome_leg_pholus::EllTrajRtResponse& res)
// {

//     _traj_par_callback_trigger = true; // to signal to other methods that a callback was received
    
//     // Resetting parameters' trajectory time 
//     _time_traj_par = 0;

//     // Assigning read trajectory parameters to target variables
//     _t_exec_traj_trgt = abs(req.t_exec) < abs(_t_exec_lb) ? abs(_t_exec_lb): abs(req.t_exec);
//     _x_c_ellps_trgt = req.x_c;
//     _z_c_ellps_trgt = req.z_c;
//     _a_ellps_trgt = _is_forward ? abs(req.a_ellps): - abs(req.a_ellps); 
//     _b_ellps_trgt = abs(req.b_ellps);
//     _alpha_trgt = req.alpha;

//     _use_vel_ff = req.use_vel_ff;
//     _use_acc_ff = req.use_acc_ff;

//     // Assigning current trajectory parameters values
//     _t_exec_traj_init = _t_exec_traj;
//     _x_c_ellps_init = _x_c_ellps;
//     _z_c_ellps_init = _z_c_ellps;
//     _a_ellps_init = _a_ellps;
//     _b_ellps_init = _b_ellps;
//     _alpha_init = _alpha;
    
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "\n Received new elliptical trajectory configuration: \n");

//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "t_exec: {}\n", _t_exec_traj_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "a_ellps: {}\n", _a_ellps_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "b_ellps: {}\n", _b_ellps_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "x_c_ellps: {}\n", _x_c_ellps_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "z_c_ellps: {}\n", _z_c_ellps_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "alpha: {}\n", _alpha_trgt);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "use velocity feedforward: {}\n", _use_vel_ff);
//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                    "use acceleration feedforward  {}\n", _use_acc_ff);

//     jhigh().jprint(fmt::fg(fmt::terminal_color::magenta), "\n");


//     res.message = "You have just sent a new trajectory configuration!";
//     res.success = true;

//     return true;
// }

void MatReplayerRt::peisekah_transition()
{

}

bool MatReplayerRt::on_initialize()
{    

    // create a nodehandle with namespace equal to
    // the plugin name
    ros::NodeHandle nh(getName());
    // use it to create our 'RosSupport' class
    // note: this must run *after* ros::init, which
    // at this point is guaranteed to be true
    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    // _ell_traj_srv = _ros->advertiseService(
    //     "my_ell_traj_srvc",
    //     &MatReplayerRt::on_ell_traj_recv_srv,
    //     this,
    //     &_queue);
    
    // Getting nominal control period from plugin method
    _dt = getPeriodSec();

    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    _n_jnts_robot = _robot->getJointNum();

    // Initializing ros server and spawning the non rt thread
    create_ros_api();
    spawn_rnt_thread();

    // Reading joint effort limits (used for saturating the trajectory)
    _model->getEffortLimits(_effort_lims);
    
    _stiffness = Eigen::VectorXd::Zero(_n_jnts_model);
    _damping = Eigen::VectorXd::Zero(_n_jnts_model);

    return true;
}

void MatReplayerRt::starting()
{
    // Resetting flag for reaching the initial tip position
    _first_run = true;

    // Creating a logger for post-processing
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    // opt.load_file_from_path = false; 
    _logger = MatLogger2::MakeLogger("/tmp/CartesioEllConfigRt_log", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Just a test to read mat files
    // XBot::MatLogger2::Options opts2;
    // opts2.load_file_from_path = true;
    // opt.enable_compression = true; // enable ZLIB compression

    // std::string mat_path = "/tmp/my_log.mat";
    // auto logger2 = XBot::MatLogger2::MakeLogger(mat_path, opts2);

    // std::vector<std::string> var_names;

    // logger2->get_mat_var_names(var_names);
    
    // Initializing logger fields to improve rt performance

    auto dscrptn_files_cell = XBot::matlogger2::MatData::make_cell(2);
    dscrptn_files_cell[0] = _urdf_path;
    dscrptn_files_cell[1] = _srdf_path;

    // initializing time (used for interpolation of trajectories inside CartesIO)
    _time = 0.0;
    // initializing time (used for interpolation of trajectory parameters)
    _time_traj_par = 0.0;

    // Update the model with the current robot state
    update_state();  

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // signal nrt thread that rt is active
    _rt_active = true;

    // Move on to run()
    start_completed();

}

void MatReplayerRt::run()
{   
    if (_first_run)
    { // first time entering the run --> smooth transition from the initial state

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

    // Update the measured state
    update_state();
    
    // Read the joint efforts computed via CartesIO (computed using acceleration_support)
    _model->getJointEffort(_effort_command);
    _robot->getJointEffort(_meas_effort);
    _effort_command += _bias_tau; // adding torque bias

    // Check input for bound violations
    saturate_input(); 

    // Set the commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command);
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

}

void MatReplayerRt::on_stop()
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

    // Get again parameters from YAML, so that next time the plugin starts with the default trajectory
    get_params_from_config();
}

void MatReplayerRt::stopping()
{
    _rt_active = false;
    stop_completed();
}

void MatReplayerRt::on_abort()
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

    _rt_active = false;
    _nrt_exit = true;
}

void MatReplayerRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_thread) _nrt_thread->join();
}

XBOT2_REGISTER_PLUGIN(MatReplayerRt, mat_replayer_rt)