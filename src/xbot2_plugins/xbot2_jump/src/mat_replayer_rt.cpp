#include "mat_replayer_rt.h"

#include <math.h> 

void MatReplayerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock

    _pause_time = 0.0;
    _approach_traj_time = 0.0;
    _smooth_imp_time = 0.0;
}

void MatReplayerRt::init_vars()
{
    _q_p_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_safe_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_trgt_appr_traj = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_init_appr_traj =  Eigen::VectorXd::Zero(_n_jnts_robot);

    _meas_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);

    _ramp_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_strt_damping = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ramp_strt_stiffness = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _f_contact_ref = Eigen::VectorXd::Zero(3);

    _tip_abs_position = Eigen::VectorXd::Zero(3);

    // states and contact force used for force estimation
    _q_p_ft_est = Eigen::VectorXd::Zero(_n_jnts_model_ft_est);
    _q_p_dot_ft_est = Eigen::VectorXd::Zero(_n_jnts_model_ft_est);
    _q_p_dot_ft_est_prev = Eigen::VectorXd::Zero(_n_jnts_model_ft_est);
    _q_p_ddot_ft_est = Eigen::VectorXd::Zero(_n_jnts_model_ft_est);
    _tau_ft_est = Eigen::VectorXd::Zero(_n_jnts_model_ft_est);

    _f_cont_est = Eigen::VectorXd::Zero(3);
    _f_cont_meas = Eigen::VectorXd::Zero(3);

}

void MatReplayerRt::reset_flags()
{

    _approach_traj_started = false;
    _approach_traj_finished = false;

    _imp_traj_started = false;
    _imp_traj_finished = false;

    _traj_started = false;
    _traj_finished = false;

    _pause_started = false;
    _pause_finished = false;

    _jump = false;

    _sample_index = 0.0; // resetting samples index, in case the plugin stopped and started again

    _approach_traj_time = 0.0;

    _smooth_imp_time = 0.0;
    
    _jump_now = false;
    
    _is_first_trigger = false;

}

void MatReplayerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;
    
    if(_pause_started && !_pause_finished)
    {
        _pause_time += _plugin_dt;
    }

    if(_imp_traj_started && !_imp_traj_finished)
    {
        _smooth_imp_time += _plugin_dt;
    }

    if(_imp_traj_finished && _approach_traj_started && !_approach_traj_finished && _jump)
    {
        _approach_traj_time += _plugin_dt;
    }

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

    if(_pause_time >= _traj_pause_time)
    {
        _pause_finished = true;
        _pause_time = _pause_time - _traj_pause_time;
    }

    if(_approach_traj_finished)
    {
        _approach_traj_time = _approach_traj_exec_time;
    }

    if(_imp_traj_finished)
    {
        _smooth_imp_time = _imp_ramp_time;
    }
    
}

void MatReplayerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 
    _srdf_path = getParamOrThrow<std::string>("~srdf_path"); 

    _urdf_path_ft_est = getParamOrThrow<std::string>("~urdf_path_ft_est");
    _srdf_path_ft_est = getParamOrThrow<std::string>("~srdf_path_ft_est");

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _is_first_jnt_passive = getParamOrThrow<bool>("~is_first_jnt_passive"); 
    _resample = getParamOrThrow<bool>("~resample"); 
    _stop_stiffness = getParamOrThrow<Eigen::VectorXd>("~stop_stiffness");
    _stop_damping = getParamOrThrow<Eigen::VectorXd>("~stop_damping");
    _delta_effort_lim = getParamOrThrow<double>("~delta_effort_lim");
    _approach_traj_exec_time = getParamOrThrow<double>("~approach_traj_exec_time");
    // _cntrl_mode =  getParamOrThrow<Eigen::VectorXd>("~cntrl_mode");

    _replay_stiffness = getParamOrThrow<Eigen::VectorXd>("~replay_stiffness"); 
    _replay_damping = getParamOrThrow<Eigen::VectorXd>("~replay_damping");
    _touchdown_stiffness = getParamOrThrow<Eigen::VectorXd>("~touchdown_stiffness"); 
    _touchdown_damping = getParamOrThrow<Eigen::VectorXd>("~touchdown_damping"); 

   _traj_pause_time = getParamOrThrow<double>("~traj_pause_time");
    _send_pos_ref = getParamOrThrow<bool>("~send_pos_ref");
    _send_vel_ref = getParamOrThrow<bool>("~send_vel_ref");
    _send_eff_ref = getParamOrThrow<bool>("~send_eff_ref");

    _tip_link_name = getParamOrThrow<std::string>("~tip_link_name"); 
    _base_link_name = getParamOrThrow<std::string>("~base_link_name");

    _imp_ramp_time = getParamOrThrow<double>("~imp_ramp_time");

    _reduce_dumped_sol_size = getParamOrThrow<bool>("~reduce_dumped_sol_size");

    _send_whole_traj = getParamOrThrow<bool>("~send_whole_traj");

    _verbose = getParamOrThrow<bool>("~verbose");

    _tip_fts_name = getParamOrThrow<std::string>("~tip_fts_name");

    _contact_linkname = getParamOrThrow<std::string>("~contact_linkname");

}

void MatReplayerRt::is_sim(std::string sim_string = "sim")
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

void MatReplayerRt::init_model_interface()
{
    // test model
    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path);
    xbot_cfg.set_srdf_path(_srdf_path);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // force estimation model
    XBot::ConfigOptions xbot_cfg_ft_est;
    xbot_cfg_ft_est.set_urdf_path(_urdf_path_ft_est);
    xbot_cfg_ft_est.set_srdf_path(_srdf_path_ft_est);
    xbot_cfg_ft_est.generate_jidmap();
    xbot_cfg_ft_est.set_parameter("is_model_floating_base", false);
    xbot_cfg_ft_est.set_parameter<std::string>("model_type", "RBDL");

    // Initializing XBot2 ModelInterface for the test model
    _model = XBot::ModelInterface::getModel(xbot_cfg); 
    _n_jnts_model = _model->getJointNum();

    // Initializing XBot2 ModelInterface for the test model
    _model_ft_est = XBot::ModelInterface::getModel(xbot_cfg_ft_est);
    _n_jnts_model_ft_est = _model_ft_est->getJointNum();
}

void MatReplayerRt::init_ft_sensor()
{

  if(_is_sim)
  { // fts only available in simulation (for now)

      _ft_sensor = _robot->getDevices<Hal::ForceTorque>().get_device(_tip_fts_name);

  }


}

void MatReplayerRt::init_ft_estimator()
{

  _ft_estimator = ContactEstimation::MakeEstimator(_contact_linkname,
                                                   _contact_dofs,
                                                   _model); // create
  // the force estimator

}

void MatReplayerRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "mr");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "mat_replayer_rt");
}

void MatReplayerRt::get_passive_jnt_est(double& pssv_jnt_pos,
                                        double& pssv_jnt_vel,
                                        double& pssv_jnt_acc)
{
    Eigen::VectorXd base_link_pos_est = _base_link_abs.translation(); // for now use
    // ground truth from Gazebo
    Eigen::VectorXd base_link_tvel_est = _base_link_vel; // for now use

    Eigen::VectorXd base_link_pos = _base_link_abs_est.translation();

    pssv_jnt_pos = base_link_pos(base_link_pos.size() - 1);

    pssv_jnt_vel = base_link_tvel_est(base_link_tvel_est.size() - 1);

    pssv_jnt_acc = (pssv_jnt_vel -
                    _q_p_dot_ft_est_prev(0) ) / _plugin_dt; // numerical diff

}

void MatReplayerRt::update_state_estimates()
{
    double passive_jnt_pos, passive_jnt_vel, pssv_jnt_acc;

    get_passive_jnt_est(passive_jnt_pos, passive_jnt_vel, pssv_jnt_acc);

    _q_p_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_ft_est(0) = passive_jnt_pos; // assign passive dofs

    _q_p_dot_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_dot_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_dot_ft_est(0) = passive_jnt_vel; // assign passive dofs

    _q_p_ddot_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = ( _q_p_dot_meas - _q_p_dot_ft_est_prev.tail(_n_jnts_robot) ) / _plugin_dt; // assign actuated dofs with meas.
    // from encoders
    _q_p_dot_ft_est(0) = pssv_jnt_acc; // assign passive dofs

    _tau_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_meas; // assign actuated dofs with meas.
    // from encoders
    _tau_ft_est(0) = 0; // no effort on passive joints

}

void MatReplayerRt::update_state()
{    
    _q_p_dot_ft_est_prev = _q_p_dot_ft_est; // getting estimated  joint
    // velocity before new state sensing

    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    _robot->getStiffness(_meas_stiffness); // used by the smooth imp. transitioner
    _robot->getDamping(_meas_damping);

    if (_is_sim)
    { // if in sim, update the ground truth force

      get_fts_force();

    }

    // Updating the test model with the measurements
    _model->setJointPosition(_q_p_meas);
    _model->setJointVelocity(_q_p_dot_meas);
    _model->update();

    _model->getPose(_tip_link_name, _base_link_name, _tip_pose_rel_base_link);

    // Updating the force estimation model with the measurements:
    // we have "exact" meas. for the actuated joints and the rest
    // (i.e. the sliding guide) is to estimated somehow

    update_state_estimates(); // updates state estimates
    // (q, q_dot, q_ddot) and efforts

    _model_ft_est->setJointPosition(_q_p_ft_est); // update the state
    _model_ft_est->setJointVelocity(_q_p_dot_ft_est);
    _model_ft_est->setJointAcceleration(_q_p_ddot_ft_est); // update joint accelerations
    _model_ft_est->setJointEffort(_tau_ft_est); // update joint efforts

    _model_ft_est->update(); // update the model

    _ft_estimator->update_estimate(); // we can now update the
    // force estimation

    _f_cont_est = _ft_estimator->get_f(); // getting the estimated contact force

    // getting estimates of the tip and hip position
    // based on the reconstructed state used to update
    // the force estimation model
    _model_ft_est->getPose(_tip_link_name, _tip_pose_abs_est);
    _model_ft_est->getPose(_base_link_name, _base_link_abs_est);

}

void MatReplayerRt::send_cmds()
{
    // always set impedance to the last setpoint to avoid issues
    _robot->setStiffness(_stiffness_setpoint); 
    _robot->setDamping(_damping_setpoint);

    if (_is_first_jnt_passive)
    { // send the last _n_jnts_robot components
        
        if (_send_eff_ref)
        {
            _robot->setEffortReference(_tau_cmd.tail(_n_jnts_robot));
        }

        if(_send_pos_ref)
        {  

            _robot->setPositionReference(_q_p_cmd.tail(_n_jnts_robot));
        }

        if(_send_vel_ref)
        {  

            _robot->setVelocityReference(_q_p_dot_cmd.tail(_n_jnts_robot));
        }

    }
    else{
        
        if (_send_eff_ref)
        {
            _robot->setEffortReference(_tau_cmd);
        }

        if(_send_pos_ref)
        {  

            _robot->setPositionReference(_q_p_cmd);
        }

        if(_send_vel_ref)
        {  

            _robot->setVelocityReference(_q_p_dot_cmd);
        }

    }
    
    _robot->move();
}

void MatReplayerRt::get_abs_tip_position()
{

    _tip_abs_position = _tip_pose_abs.translation();

}

void MatReplayerRt::get_fts_force()
{
  if(_is_sim)
  { // fts only available in simulation (for now)

    Eigen::Vector6d wrench = _ft_sensor->getWrench();

    _meas_tip_f_loc = wrench.head(3);
    _meas_tip_t_loc = wrench.tail(3);

    // rotating the force into world frame
    // then from base to world orientation

    _meas_tip_f_abs = _tip_pose_abs * _meas_tip_f_loc;

  }

}

void MatReplayerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/MatReplayerRt", opt); // date-time automatically appended
    }
    else
    {
        if (!_is_sim)
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        else
        {
            _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
        }
        
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("stop_stiffness", _stop_stiffness);
    _dump_logger->add("stop_damping", _stop_damping);
    _dump_logger->add("is_sim", int(_is_sim));
    
    _dump_logger->add("send_pos_ref", int(_send_pos_ref));
    _dump_logger->add("send_vel_ref", int(_send_vel_ref));
    _dump_logger->add("send_eff_ref", int(_send_eff_ref));

    // _dump_logger->create("plugin_time", 1);
    _dump_logger->create("jump_replay_times", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_time", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("replay_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_stiffness", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("meas_damping", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_meas", _n_jnts_robot), 1, _matlogger_buffer_size;
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("f_contact_ref", 3, 1, _matlogger_buffer_size);
    _dump_logger->create("f_contact_meas", 3, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
      _dump_logger->create("tip_pos_meas", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_vel", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_omega", 3, 1, _matlogger_buffer_size);

      _dump_logger->create("q_p_ft_est", _n_jnts_model_ft_est), 1, _matlogger_buffer_size;
      _dump_logger->create("q_p_dot_ft_est", _n_jnts_model_ft_est, 1, _matlogger_buffer_size);
      _dump_logger->create("q_p_ddot_ft_est", _n_jnts_model_ft_est, 1, _matlogger_buffer_size);
      _dump_logger->create("tau_ft_est", _n_jnts_model_ft_est, 1, _matlogger_buffer_size);
      _dump_logger->create("f_cont_est", 3, 1, _matlogger_buffer_size);
    }

    _dump_logger->create("tip_pos_rel_base_link", 3, 1, _matlogger_buffer_size);

    // auto dscrptn_files_cell = XBot::matlogger2::MatData::make_cell(4);
    // dscrptn_files_cell[0] = _mat_path;
    // dscrptn_files_cell[1] = _mat_name;
    // dscrptn_files_cell[2] = _robot->getUrdfPath();
    // dscrptn_files_cell[3] = _robot->getSrdfPath();
    // _dump_logger->save("description_files", dscrptn_files_cell);

}

void MatReplayerRt::add_data2dump_logger()
{

    if (_is_first_jnt_passive)
    { // remove first joint from logged commands

        _dump_logger->add("q_p_cmd", _q_p_cmd.tail(_n_jnts_robot));
        _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd.tail(_n_jnts_robot));
        _dump_logger->add("tau_cmd", _tau_cmd.tail(_n_jnts_robot));

    }
    else
    {

        _dump_logger->add("q_p_cmd", _q_p_cmd);
        _dump_logger->add("q_p_dot_cmd", _q_p_dot_cmd);
        _dump_logger->add("tau_cmd", _tau_cmd);

    }

    _dump_logger->add("replay_stiffness", _replay_stiffness);
    _dump_logger->add("replay_damping", _replay_damping);
    _dump_logger->add("meas_stiffness", _meas_stiffness);
    _dump_logger->add("meas_damping", _meas_damping);

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

    _dump_logger->add("f_contact_ref", _f_contact_ref);

    _dump_logger->add("replay_time", _loop_time);

    _dump_logger->add("tip_pos_rel_base_link", _tip_pose_rel_base_link.translation());

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
        _dump_logger->add("tip_pos_meas", _tip_abs_position);

        _dump_logger->add("base_link_abs", _base_link_abs.translation());

        _dump_logger->add("base_link_vel", _base_link_vel);

        _dump_logger->add("base_link_omega", _base_link_omega);

        _dump_logger->add("meas_tip_f_loc", _meas_tip_f_loc);

        _dump_logger->add("meas_tip_f_abs", _meas_tip_f_abs);


        _dump_logger->add("q_p_ft_est", _q_p_ft_est);

        _dump_logger->add("q_p_dot_ft_est", _q_p_dot_ft_est);

        _dump_logger->add("q_p_ddot_ft_est", _q_p_ddot_ft_est);

        _dump_logger->add("tau_ft_est", _tau_ft_est);

        _dump_logger->add("f_cont_est", _f_cont_est);

    }

}

void MatReplayerRt::add_data2bedumped()
{
    
    // _dump_logger->add("plugin_time", _loop_time)

    if (_reduce_dumped_sol_size)
    {  // only adding data when replaying trajectory to save memory

        if (_traj_started && !_traj_finished)
        { // trajectory is being published
            
            if (_sample_index <= (_traj.get_n_nodes() - 1))
            {

                add_data2dump_logger();

            }

        }
    }
    else
    { // dump data at each loop cycle

         add_data2dump_logger();

    }
    
    

}

void MatReplayerRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Service server */
    _jump_now_srv = _ros->advertiseService(
        "my_jump_now",
        &MatReplayerRt::on_jump_msg_rcvd,
        this,
        &_queue);

    /* Subscribers */
    _base_link_pose_sub = _ros->subscribe("/xbotcore/link_state/base_link/pose",
                                &MatReplayerRt::on_base_link_pose_received,
                                this,
                                1,  // queue size
                                &_queue);

    _base_link_twist_sub = _ros->subscribe("/xbotcore/link_state/base_link/twist",
                                &MatReplayerRt::on_base_link_twist_received,
                                this,
                                1,  // queue size
                                &_queue);

    /* Publishers */
    awesome_leg::MatReplayerStatus replay_st_prealloc;
    replay_st_prealloc.approach_traj_finished = false;
    replay_st_prealloc.traj_finished = false;

    _replay_status_pub = _ros->advertise<awesome_leg::MatReplayerStatus>(
        "replay_status_node", 1, replay_st_prealloc);


}

int MatReplayerRt::was_jump_signal_received()
{
    int res = -1;

    if (_jump)
    {
        _is_first_trigger = !_is_first_trigger;
        
        if (!_approach_traj_started && !_traj_started && _is_first_trigger)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                "\n Initializing approach trajectory sequence...\n Please wait for the robot to stop, place it on the ground and clear the jumping area.\n");

            res = 1;

            _q_p_safe_cmd = _q_p_meas; // initial position for the approach traj.
            _q_p_trgt_appr_traj = _q_p_ref.block(1, 0, _n_jnts_robot, 1); // target pos. for the approach traj

            _imp_traj_started = true; // start impedance traj
            _approach_traj_started = false; // will wait for imp. traj to finish
            // before starting the approach trajectory

            _ramp_strt_stiffness = _meas_stiffness;
            _ramp_strt_damping = _meas_damping;
        }
        
        if (_approach_traj_finished && !_traj_started && !_is_first_trigger)
        {
            
            _traj_started = true; // send actual trajectory

            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                "\n Starting jump sequence! Please clear the jumping area!\n");

            res = 2;

            _sample_index = 0; // will start sending the loaded trajectory
            
            _dump_logger->add("jump_replay_times", _loop_time);
            
        }

    }

    return res;
}

bool MatReplayerRt::on_jump_msg_rcvd(const awesome_leg::JumpNowRequest& req,
                    awesome_leg::JumpNowResponse& res)
{

    _jump = req.jump_now; // setting jump flag

    std::string message;
    
    _jump_phase_state = was_jump_signal_received();

    res.message = (_jump_phase_state == 1) ? "Starting replaying of approach trajectory!" : "Starting replaying of jump trajectory!";

    res.success = true;

    return res.success; 
}

void MatReplayerRt::on_base_link_pose_received(const geometry_msgs::PoseStamped& msg)
{   

    tf::poseMsgToEigen(msg.pose, _base_link_abs);

    _tip_pose_abs = _base_link_abs * _tip_pose_rel_base_link; // from tip to base link orientation,

    get_abs_tip_position();
}

void MatReplayerRt::on_base_link_twist_received(const geometry_msgs::TwistStamped& msg)
{

    Eigen::Vector6d twist;

    tf::twistMsgToEigen(msg.twist, twist);

    _base_link_vel = twist.head(3);
    _base_link_omega = twist.tail(3);
}

void MatReplayerRt::load_opt_data()
{   

    _traj = plugin_utils::TrajLoader(_mat_path + _mat_name, true, 0.0001, false);

    int n_traj_jnts = _traj.get_n_jnts();

    if(n_traj_jnts != _n_jnts_robot) 
    {
        jwarn("The loaded trajectory has {} joints, while the robot has {} .\n Make sure to somehow select the right components!!",
        n_traj_jnts, _n_jnts_robot);
    }

    if (_resample)
    { // resample input data at the plugin frequency (for now it sucks)

        _traj.resample(_plugin_dt, _q_p_ref, _q_p_dot_ref, _tau_ref, _f_cont_ref); // just brute for linear interpolation for now (for safety, better to always use the same plugin_dt as the loaded trajectory)

    }
    else
    { // load raw data without changes

        Eigen::MatrixXd dt_opt;

        _traj.get_loaded_traj(_q_p_ref, _q_p_dot_ref, _tau_ref, dt_opt, _f_cont_ref);

        jwarn("The loaded trajectory was generated with a dt of {} s, while the rt plugin runs at {} .\n ",
        dt_opt(0), _plugin_dt);

    }

    _takeoff_index = _traj.get_takeoff_index(); // takeoff index

}

void MatReplayerRt::saturate_effort()
{
    int input_sign = 1; // defaults to positive sign 

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        if (abs(_tau_cmd[i]) >= abs(_effort_lims[i]))
        {
            input_sign = (signbit(_tau_cmd[i])) ? -1: 1; 

            _tau_cmd[i] = input_sign * (abs(_effort_lims[i]) - _delta_effort_lim);
        }
    }
}

void MatReplayerRt::ramp_imp_smoothly()
{

    double phase = _smooth_imp_time / _imp_ramp_time;

    _ramp_stiffness = _peisekah_utils.compute_peisekah_vect_val(phase, _ramp_strt_stiffness, _replay_stiffness);

    _ramp_damping = _peisekah_utils.compute_peisekah_vect_val(phase, _ramp_strt_damping, _replay_damping);

    _q_p_cmd = _q_p_safe_cmd; // enforce reference to a "safe" state

    _stiffness_setpoint = _ramp_stiffness; 
    _damping_setpoint = _ramp_damping;

}

void MatReplayerRt::set_approach_trajectory()
{

    double phase = _approach_traj_time / _approach_traj_exec_time; // phase ([0, 1] inside the approach traj)

    _q_p_cmd = _peisekah_utils.compute_peisekah_vect_val(phase, _q_p_safe_cmd, _q_p_trgt_appr_traj);

    _stiffness_setpoint = _replay_stiffness; 
    _damping_setpoint = _replay_damping;
    
}

void MatReplayerRt::pub_replay_status()
{
    auto status_msg = _replay_status_pub->loanMessage();
    status_msg->msg().approach_traj_finished = _approach_traj_finished;
    status_msg->msg().traj_finished = _traj_finished;

    _replay_status_pub->publishLoaned(std::move(status_msg));
}

void MatReplayerRt::set_trajectory()
{ // always called in each plugin loop
  // is made of a number of phases, each signaled by suitable flags
  // remember to increase the sample index at the end of each phase, 
  // if necessary

//    if (_is_first_run)
//    { // set impedance vals and pos ref to safe values at first plugin loop

//        if (_verbose)
//        {
//            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
//                       "\n (first run) \n");
//        }

//        _ramp_strt_stiffness = _meas_stiffness;
//        _ramp_strt_damping = _meas_damping;

//        _q_p_safe_cmd = _q_p_meas; // initial position for the approach traj.
//        // upon plugin start

//        _imp_traj_started = true; // ramp impedance to target values smoothly when
//        //starting the plugin the first time

//        _sample_index++; // incrementing loop counter
//    }

    if (_imp_traj_started && !_imp_traj_finished)
    { // still ramping (up) impedance
        
        if (_smooth_imp_time > _imp_ramp_time - 0.000001)
        {
            _imp_traj_finished = true; // finished ramping imp.

            _approach_traj_started = true; // start publishing approach
            // trajectory

            _stiffness_setpoint = _replay_stiffness; 
            _damping_setpoint = _replay_damping;

            _q_p_init_appr_traj = _q_p_cmd; // setting initial approach traj. point
            // to last sent position command

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\n Joint impedance successfully ramped to target \n");
        }
        else
        {
            ramp_imp_smoothly();
        }

    }

    if (_imp_traj_finished && _approach_traj_started && !_approach_traj_finished && _jump)
    { // still publishing the approach trajectory

        if (_approach_traj_time > _approach_traj_exec_time - 0.000001)
        {
            _approach_traj_finished = true; // finished approach traj
            
            // start of trajectory replay is triggered by the callback in this case

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   std::string("\n Approach trajectory finished... ready to jump \n"));

        }
        else
        {
            set_approach_trajectory();
        }

        _sample_index++; // incrementing loop counter
        
    }

    if (_traj_started && !_traj_finished && _jump)
    { // publish current trajectory sample
        
        if (_sample_index <= _takeoff_index)
        { // before takeoff
            
            // by default assign all commands anyway
            _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
            _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
            _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
            _f_contact_ref = _f_cont_ref.col(_sample_index);
            
            _stiffness_setpoint = _replay_stiffness; 
            _damping_setpoint = _replay_damping;

            saturate_effort(); // perform input torque saturation
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                   "\n (before takeoff) \n");
            }
        
        }

        if (_sample_index <= (_traj.get_n_nodes() - 1) && _sample_index > _takeoff_index)
        { // after the optimized takeoff phase
            
            if (_send_whole_traj)
            {
                _q_p_cmd = _q_p_ref.col(_sample_index).tail(_n_jnts_robot);
                _q_p_dot_cmd = _q_p_dot_ref.col(_sample_index).tail(_n_jnts_robot);
                _tau_cmd = _tau_ref.col(_sample_index).tail(_n_jnts_robot);
                _f_contact_ref = _f_cont_ref.col(_sample_index);
                
                _stiffness_setpoint = _replay_stiffness; 
                _damping_setpoint = _replay_damping;

            }
            else
            {
                _q_p_cmd = _q_p_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _q_p_dot_cmd = _q_p_dot_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _tau_cmd = _tau_ref.col(_takeoff_index).tail(_n_jnts_robot);
                _f_contact_ref = _f_cont_ref.col(_takeoff_index);

                _stiffness_setpoint = _touchdown_stiffness; 
                _damping_setpoint = _touchdown_damping;

            }
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                    "\n (after takeoff) \n");
            }
            saturate_effort(); // perform input torque saturation
            
        }

        if (_sample_index > (_traj.get_n_nodes() - 1))
        { // reached the end of the trajectory
            
            if (_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n (trajectory end)\n");
            }
            _traj_finished = true;
            _sample_index = 0; // reset publish index (will be used to publish the loaded trajectory)
            
        }

        _sample_index++; // incrementing loop counter

    }

    if (_traj_finished && _jump)
    { // finished publishing trajectory

        _pause_started = true;

        if (!_pause_finished)
        { // do nothing in this control loop


        }
        else 
        {

            reset_flags(); // reset flags

            _jump = false; // directly trigger next jump sequence

            _q_p_safe_cmd = _q_p_meas; // keep position reference to currently measured state
        }

        _stiffness_setpoint = _touchdown_stiffness; 
        _damping_setpoint = _touchdown_damping;

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n Finished publishing \n");
        }
        
        _sample_index++; // incrementing loop counter
    }

    if (!_jump)
    {
        // _q_p_cmd = _q_p_safe_cmd;
        _q_p_dot_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
        _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);

        if (_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::magenta),
                       "\n Waiting for commands... \n");
        }
    }

}

bool MatReplayerRt::on_initialize()
{ 
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    // Initializing XBot2 model interface using the read parameters
    init_model_interface();

    _n_jnts_robot = _robot->getJointNum();

    init_vars();

    _robot->getEffortLimits(_effort_lims);

    init_nrt_ros_bridge();
    
    // Initializing CartesIO solver, ros server and spawning the non rt thread
    // init_cartesio_solver();
    create_ros_api();

    load_opt_data(); // load trajectory from file (to be placed here in starting because otherwise
    // a seg fault will arise)

    _peisekah_utils = plugin_utils::PeisekahTrans();

    init_ft_sensor();

    init_ft_estimator();

    return true;
    
}

void MatReplayerRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers
    
    _stiffness_setpoint = _replay_stiffness;
    _damping_setpoint = _replay_damping;

    // setting the control mode to effort + velocity + stiffness + damping
    _robot->setControlMode(ControlMode::Position() + ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() +
            ControlMode::Damping());

    update_state(); // read current jnt positions and velocities

    // Move on to run()
    start_completed();
    
}

void MatReplayerRt::run()
{  
    int i = 0;

    update_state(); // update all necessary states

    _queue.run();

    set_trajectory();

    pub_replay_status();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    send_cmds(); // send commands to the robot

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
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

    reset_flags();

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void MatReplayerRt::stopping()
{

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

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void MatReplayerRt::on_close()
{
    jinfo("Closing MatReplayerRt");
}

XBOT2_REGISTER_PLUGIN(MatReplayerRt, mat_replayer_rt)

