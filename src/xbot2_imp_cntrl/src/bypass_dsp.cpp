#include "bypass_dsp.h"

bool BypassDsp::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file (here only minimal checks are made)

    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");

    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");

    _use_motor_side_readings = getParamOrThrow<bool>("~use_motor_side_readings");

    _traj_prm_rmp_time = getParamOrThrow<double>("~traj_prm_rmp_time");
    _imp_rmp_time = getParamOrThrow<double>("~imp_rmp_time");

    _t_exec_trgt = getParamOrThrow<Eigen::VectorXd>("~t_exec");
    _t_exec_lb = getParamOrThrow<double>("~t_exec_lb");
    _center_trgt = getParamOrThrow<Eigen::VectorXd>("~center");
    _phase_off_trgt = getParamOrThrow<Eigen::VectorXd>("~phase_off");
    _overshoot_trgt = getParamOrThrow<Eigen::VectorXd>("~overshoot");

    _stiffness_trgt = getParamOrThrow<Eigen::VectorXd>("~jnt_stiffness");
    _damping_trgt = getParamOrThrow<Eigen::VectorXd>("~jnt_damping");

    _jnt_imp_lims = getParamOrThrow<Eigen::VectorXd>("~jnt_imp_lims");

    if (    (_t_exec_trgt.size() != _n_jnts_robot) ||
            (_center_trgt.size() != _n_jnts_robot) ||
         (_phase_off_trgt.size() != _n_jnts_robot) ||
         (_overshoot_trgt.size() != _n_jnts_robot) ||
         (_stiffness_trgt.size() !=  _n_jnts_robot)||
         (_damping_trgt.size() !=  _n_jnts_robot)  )
    { // checking dimensions of provided yaml parameters

        jhigh().jerror("Wrong dimension of parameters in YAML file. Please check that all of them are vector of dimension {} \n", _n_jnts_robot);
    }

    return true;
}

void BypassDsp::saturate_input()
{
    int input_sign = 1; // defaults to positive sign 

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        if (abs(_effort_command[i]) >= abs(_effort_lims[i]))
        {
            input_sign = (signbit(_effort_command[i])) ? -1: 1; 

            _effort_command[i] = input_sign * (abs(_effort_lims[i]) - _delta_effort_lim);
        }
    }
}

void BypassDsp::update_state()
{
    // "sensing" the robot
    _robot->sense();
    // Getting robot state
    if (_use_motor_side_readings)
    {
        _robot->getMotorPosition(_q_p_meas);
        _robot->getMotorVelocity(_q_p_dot_meas); 
    }

    _robot->getJointPosition(_q_p_meas);
    _robot->getJointVelocity(_q_p_dot_meas); 
       
}

void BypassDsp::compute_ref_traj(Eigen::VectorXd time)
{
    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _q_p_trgt[i] = _center[i] + _overshoot[i] * sin(2 * M_PI * time[i]/_t_exec[i] + _phase_off[i]);
        _q_p_dot_trgt[i] = 2 * M_PI / _t_exec[i] * _overshoot[i] * cos(2 * M_PI * time[i]/_t_exec[i] + _phase_off[i]);
    }
}

void BypassDsp::peisekah_transition() 
{
    // Processing transition for parameters trajectory
    // (all parameters are ramped to their targets within the same ramp time)

    if (_traj_par_callback_trigger) // a new traj. config was received (defaults to false)
    {
        for (int i = 0; i < _n_jnts_robot; i++) // loop through every joint
        {   
            if (_time_traj_par >= _traj_prm_rmp_time) // parameters transition completed --> set params to their nominal target to avoid numerical errors
            {
                for (int i = 0; i < _n_jnts_robot; i++)
                {
                    _t_exec[i] = _t_exec_trgt[i];
                    _center[i] = _center_trgt[i];
                    _phase_off[i] = _phase_off_trgt[i];
                    _overshoot[i] = _overshoot_trgt[i];
                }

                _traj_par_callback_trigger = false; // reset callback counter, since parameters' trajectory has terminated
            }
            else
            {
                // Smooth transition for traj parameters (9th order polinomial traj.)
                double common_part_traj = (126.0 * pow(_time_traj_par/_traj_prm_rmp_time, 5) - 420.0 * pow(_time_traj_par/_traj_prm_rmp_time, 6) + 540.0 * pow(_time_traj_par/_traj_prm_rmp_time, 7) - 315.0 * pow(_time_traj_par/_traj_prm_rmp_time, 8) + 70.0 * pow(_time_traj_par/_traj_prm_rmp_time, 9));

                for (int i = 0; i < _n_jnts_robot; i++)
                {
                    _t_exec[i] = _t_exec_init[i] + (_t_exec_trgt[i] - _t_exec_init[i]) *  common_part_traj;
                    _center[i] = _center_init[i] + (_center_trgt[i] - _center_init[i]) *  common_part_traj;
                    _phase_off[i] = _phase_off_init[i] + (_phase_off_trgt[i] - _phase_off_init[i]) *  common_part_traj;
                    _overshoot[i] = _overshoot_init[i] + (_overshoot_trgt[i] - _overshoot_init[i]) *  common_part_traj;

                }
            }
        }
        _time_traj_par += _dt; // incrementing trajectory parameter time
    }

    // Processing transitions for impedance setpoints

    if (_imp_callback_trigger) // a new traj. config was received (defaults to false)
    {
        for (int i = 0; i < _n_jnts_robot; i++) // loop through every joint
        {   
            if (_time_jnt_imp >= _imp_rmp_time) // parameters transition completed --> set params to their nominal target to avoid numerical errors
            {
                for (int i = 0; i < _n_jnts_robot; i++)
                {
                    _stiffness[i] = _stiffness_trgt[i];
                    _damping[i] = _damping_trgt[i];
                }

                _imp_callback_trigger = false; // reset callback counter, since parameters' trajectory has terminated
            }
            else
            {
                // Smooth transition for traj parameters (9th order polinomial traj.)
                double common_part_traj = (126.0 * pow(_time_jnt_imp/_imp_rmp_time, 5) - 420.0 * pow(_time_jnt_imp/_imp_rmp_time, 6) + 540.0 * pow(_time_jnt_imp/_imp_rmp_time, 7) - 315.0 * pow(_time_jnt_imp/_imp_rmp_time, 8) + 70.0 * pow(_time_jnt_imp/_imp_rmp_time, 9));

                for (int i = 0; i < _n_jnts_robot; i++)
                {
                    _stiffness[i] = _stiffness_init[i] + (_stiffness_trgt[i] - _stiffness_init[i]) *  common_part_traj;
                    _damping[i] = _damping_init[i] + (_damping_trgt[i] - _damping_init[i]) *  common_part_traj;

                }
            }
        }
        _time_jnt_imp += _dt; // incrementing trajectory parameter time
    }
}

bool BypassDsp::on_sin_traj_recv_srv(const awesome_leg_pholus::SinJointTrajRequest& req,
                          awesome_leg_pholus::SinJointTrajResponse& res)
{

    _traj_par_callback_trigger = true; // to signal to other methods that a callback was received
    
    // Resetting parameters' trajectory time 
    _time_traj_par = 0;

    if (    (req.t_exec.size() != _n_jnts_robot) ||
            (req.center.size() != _n_jnts_robot) ||
         (req.phase_off.size() != _n_jnts_robot) ||
         (req.overshoot.size() != _n_jnts_robot)    )
    { // checking dimensions of provided yaml trajectory configuration

        jhigh().jerror("Wrong dimension of trajectory parameters passed via ROS service. Please check that all of them are vector of dimension {} \n", _n_jnts_robot);

        res.success = false;

        return res.success;
    }

    // Assigning read trajectory parameters to target variables
    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _t_exec_trgt[i] = abs(req.t_exec[i]) < abs(_t_exec_lb) ? abs(_t_exec_lb) : abs(req.t_exec[i]); // first check the execution time

        // checking for joint position/velocity bound violations --> if bounds violated do not set the new trajectory
        if ( (req.center[i] - abs(req.overshoot[i])) < _q_min[i] || ((req.center[i] + abs(req.overshoot[i])) > _q_max[i])  || 
             2 * M_PI /  _t_exec_trgt[i] * abs(_overshoot[i]) > _q_dot_max[i] )
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n The received traj. configuration exceeds joint position OR velocity limits.\n The trajectory will not be set.");

            res.message = "Required trajectory exceeds joint limits!";

            res.success = false;

            _traj_par_callback_trigger = false; // do not signal trajectory change to other parts of the code

            return res.success;
        }

        _center_trgt[i] = req.center[i];
        _overshoot_trgt[i] = abs(req.overshoot[i]);

        _phase_off_trgt[i] = req.phase_off[i];
    }

    // Assigning current trajectory parameters values
    _t_exec_init = _t_exec;
    _center_init = _center;
    _phase_off_init = _phase_off;
    _overshoot_init = _overshoot;
    
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n Setting joint sinusoidal trajectory configuration to: \n");

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "t_exec: {}\n", _t_exec_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "_center: {}\n", _center_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "_phase_off: {}\n", _phase_off_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "_overshoot: {}\n", _overshoot_trgt);               
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta), "\n");

    res.message = "You have just sent a new trajectory configuration!";

    res.success = true;

    return res.success;
}

bool BypassDsp::on_jnt_imp_setpoint_recv_srv(const awesome_leg_pholus::BypassDspRtRequest& req,
                          awesome_leg_pholus::BypassDspRtResponse& res)
{   
    _imp_callback_trigger = true; // to signal to other methods that a callback was received

    // Resetting parameters' trajectory time 
    _time_jnt_imp = 0;

    if (    (req.jnt_stiffness_setpoint.size() != _n_jnts_robot) ||
           (req.jnt_damping_setpoint.size() != _n_jnts_robot)      )
    { // checking dimensions of provided yaml trajectory configuration

        jhigh().jerror("Wrong dimension of impedance setpoints passed via ROS service. Please check that both stiffness and damping are vectors of dimension {} \n", _n_jnts_robot);

        res.success = false;

        return res.success;
    }

    // Assigning read trajectory parameters to target variables
    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _stiffness_trgt[i] = abs(req.jnt_stiffness_setpoint[i]) > abs(_jnt_imp_lims[0]) ? abs(_jnt_imp_lims[0]) : abs(req.jnt_stiffness_setpoint[i]); // assign value checking impedance limits violations
        _damping_trgt[i] = abs(req.jnt_damping_setpoint[i]) > abs(_jnt_imp_lims[1]) ? abs(_jnt_imp_lims[1]) : abs(req.jnt_damping_setpoint[i]);
    
    }

    // Assigning current impedance
    _stiffness_init = _stiffness;
    _damping_init = _damping;

    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _stiffness[i] = req.jnt_stiffness_setpoint[i];
        _damping[i] = req.jnt_damping_setpoint[i];
    }

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n Received new joint impedance setpoints: \n");

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "stiffness: {}\n", _stiffness_trgt);
    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "damping: {}\n", _damping_trgt);

    jhigh().jprint(fmt::fg(fmt::terminal_color::magenta), "\n");

    res.message = "You have just sent a new joint impedance setpoint!";
    res.success = true;

    return res.success;
}

void BypassDsp::compute_jnt_imp_cntrl()
{   
    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _effort_command[i] = - _stiffness[i] * (_q_p_meas[i] - _q_p_trgt[i]) - _damping[i] * (_q_p_dot_meas[i] - _q_p_dot_trgt[i]);
    }
}

bool BypassDsp::on_initialize()
{    

    _n_jnts_robot = _robot->getJointNum();

    _robot->getJointLimits(_q_min, _q_max);
    _robot->getVelocityLimits(_q_dot_max);

    // Define dimension of dinamic vectors

    _stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);
    _damping = Eigen::VectorXd::Zero(_n_jnts_robot);

    _t_exec_init = Eigen::VectorXd::Zero(_n_jnts_robot);
    _center_init = Eigen::VectorXd::Zero(_n_jnts_robot);
    _phase_off_init = Eigen::VectorXd::Zero(_n_jnts_robot);
    _overshoot_init = Eigen::VectorXd::Zero(_n_jnts_robot);

    _t_exec_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);
    _center_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);
    _phase_off_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);
    _overshoot_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _t_exec = Eigen::VectorXd::Zero(_n_jnts_robot);
    _center = Eigen::VectorXd::Zero(_n_jnts_robot);
    _phase_off = Eigen::VectorXd::Zero(_n_jnts_robot);
    _overshoot = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);
    _damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _stiffness_init = Eigen::VectorXd::Zero(_n_jnts_robot);
    _damping_init = Eigen::VectorXd::Zero(_n_jnts_robot);
    _stiffness_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);
    _damping_trgt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _effort_command = Eigen::VectorXd::Zero(_n_jnts_robot);

    // Reading joint effort limits (used for saturating the trajectory)
    _robot->getEffortLimits(_effort_lims);
    
    // create a nodehandle with namespace equal to
    // the plugin name
    ros::NodeHandle nh(getName());
    // use it to create our 'RosSupport' class
    // note: this must run *after* ros::init, which
    // at this point is guaranteed to be true
    _ros = std::make_unique<RosSupport>(nh);

    /* Service servers */
    _sin_traj_srv = _ros->advertiseService(
        "my_sin_jnt_traj_srvc",
        &BypassDsp::on_sin_traj_recv_srv,
        this,
        &_queue);
    
    _bypass_dsp_srv = _ros->advertiseService(
        "bypass_dsp_srvc",
        &BypassDsp::on_jnt_imp_setpoint_recv_srv,
        this,
        &_queue);

    // Getting nominal control period from plugin method
    _dt = getPeriodSec();

    // Reading all the necessary parameters from a configuration file
    get_params_from_config();

    return true;
}

void BypassDsp::starting()
{
    // Resetting flag for reaching the initial tip position
    _first_run = true;

    // Creating a logger for post-processing
    MatLogger2::Options opt;
    opt.default_buffer_size = 1e6; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    _logger = MatLogger2::MakeLogger("/tmp/BypassDspRt_log", opt); // date-time automatically appended
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // Initializing logger fields to improve rt performance
    
    _logger->create("meas_efforts", _n_jnts_robot);
    _logger->create("computed_efforts", _n_jnts_robot);

    _logger->create("t_exec", _n_jnts_robot);
    _logger->create("center", _n_jnts_robot);
    _logger->create("phase_off", _n_jnts_robot);
    _logger->create("overshoot", _n_jnts_robot);

    _logger->create("q_p_meas", _n_jnts_robot);
    _logger->create("q_p_dot_meas", _n_jnts_robot);

    _logger->create("q_p_trgt", _n_jnts_robot);
    _logger->create("q_p_dot_trgt", _n_jnts_robot);

    _logger->create("jnt_stiffness_setpoint", _n_jnts_robot);
    _logger->create("jnt_damping_setpoint", _n_jnts_robot);

    // initializing time vector (used for computing the joint reference trajectories)
    _time = Eigen::VectorXd::Zero(_n_jnts_robot);

    // initializing time (used for transitioning trajectory parameters and impedance setpoints)
    _time_traj_par = 0;
    _time_jnt_imp = 0;

    // Update the state variables by reading from the robot
    update_state();  

    // setting the control mode to effort + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping());

    // signal nrt thread that rt is active
    _rt_active = true;

    _logger->add("plugin_dt", _dt);
    _logger->add("stop_stiffness", _stop_stiffness);
    _logger->add("stop_damping", _stop_damping);
    _logger->add("t_exec_lb", _t_exec_lb);
    _logger->add("traj_prm_rmp_time",_traj_prm_rmp_time);
    _logger->add("use_motor_side_readings",_use_motor_side_readings);

    // Move on to run()
    start_completed();


}

void BypassDsp::run()
{   
    if (_first_run)
    { // first time entering the run --> smooth transition from the initial state

        _traj_par_callback_trigger = true; // faking a received trajectory configuration
        _imp_callback_trigger = true; // faking a received joint impedance setpoint

        update_state();

        for (int i = 0; i < _n_jnts_robot; i++)
        { // ramp from the initial state to the target and only then start the trajectory
            _t_exec_init[i] = _t_exec_trgt[i]; // does not count
            _center_init[i] = _q_p_meas[i]; // ramp to the initial target value from the current state
            _phase_off_init[i] = 0;
            _overshoot_init[i] = 0;

        }

        _robot->getStiffness(_stiffness_init); // ramp impedance values to target starting from the setpoint on the robot
        _robot->getDamping(_damping_init);

        _first_run = false;
        
    }
    
    // process callbacks
    _queue.run();
    
    // Compute a smooth transition for the (potentially) changed trajectory parameters. 
    peisekah_transition();

    // Update the measured state
    update_state();
    
    compute_ref_traj(_time);
    
    // Compute impedance cntrl effort
    compute_jnt_imp_cntrl();
    
    _robot->getJointEffort(_meas_effort);

    // Check input for bound violations
    saturate_input(); 

    // Set the commands (and also stiffness/damping)
    _robot->setEffortReference(_effort_command);
    _robot->setPositionReference(_q_p_trgt);
    _robot->setVelocityReference(_q_p_dot_trgt);
    _robot->setStiffness(Eigen::VectorXd::Zero(_n_jnts_robot));
    _robot->setDamping(Eigen::VectorXd::Zero(_n_jnts_robot));

    // Send commands to the robot
    _robot->move(); 

    // Update time(s)

    for (int i = 0; i < _n_jnts_robot; i++)
    {
        _time[i] += _dt;

        if (_time[i] >= _t_exec[i])
        {
            _time[i] = _time[i] - _t_exec[i];
        }
    }
    
    
    // Adding that useful data to logger

    _logger->add("meas_efforts", _meas_effort);
    _logger->add("computed_efforts", _effort_command);

    _logger->add("t_exec", _t_exec);
    _logger->add("center", _center);
    _logger->add("phase_off", _phase_off);
    _logger->add("overshoot", _overshoot);

    _logger->add("q_p_meas", _q_p_meas);
    _logger->add("q_p_dot_meas", _q_p_dot_meas);

    _logger->add("q_p_trgt", _q_p_trgt);
    _logger->add("q_p_dot_trgt", _q_p_dot_trgt);

    _logger->add("jnt_stiffness_setpoint", _stiffness);
    _logger->add("jnt_damping_setpoint", _damping);

}

void BypassDsp::on_stop()
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

void BypassDsp::stopping()
{
    _rt_active = false;
    stop_completed();
}

void BypassDsp::on_abort()
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

void BypassDsp::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_thread) _nrt_thread->join();
}

XBOT2_REGISTER_PLUGIN(BypassDsp, bypass_dsp_rt)