#include "contact_est_rt.h"

#include <math.h> 

void ContactEstRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void ContactEstRt::init_vars()
{
    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

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

void ContactEstRt::reset_flags()
{

}

void ContactEstRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }
    
}

void ContactEstRt::get_params_from_config()
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

    _tip_link_name = getParamOrThrow<std::string>("~tip_link_name"); 
    _base_link_name = getParamOrThrow<std::string>("~base_link_name");

    _verbose = getParamOrThrow<bool>("~verbose");

    _tip_fts_name = getParamOrThrow<std::string>("~tip_fts_name");
    _contact_linkname = getParamOrThrow<std::string>("~contact_linkname");
    _test_rig_linkname = getParamOrThrow<std::string>("~test_rig_linkname");

}

void ContactEstRt::is_sim(std::string sim_string = "sim")
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

void ContactEstRt::init_model_interface()
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

    _model_ft_est->getPose(_test_rig_linkname, _test_rig_pose);

    _test_rig_pose_inv = _test_rig_pose.inverse(); // computed here
    // to avoid rt constraint violations in the control loop
    
    // offset of the base link wrt the world link in the ft estimation
    // model

}

void ContactEstRt::init_ft_sensor()
{

  if(_is_sim)
  { // fts only available in simulation (for now)

      _ft_sensor = _robot->getDevices<Hal::ForceTorque>().get_device(_tip_fts_name);

      _ft_tip_sensor_found = (!_ft_sensor) ?  false : true; // flag to signal the
      // presence (or absence) of the tip force torque sensor (calling methods on a nullptr
      // causes a seg. fault)

  }


}

void ContactEstRt::init_ft_estimator()
{

}

void ContactEstRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "mr");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "mat_replayer_rt");
}

void ContactEstRt::get_passive_jnt_est(double& pssv_jnt_pos,
                                        double& pssv_jnt_vel,
                                        double& pssv_jnt_acc)
{

    if(_is_sim)
    { // for now, estimates only available in simulation
    _base_link_pos_rel_test_rig = _base_link_abs * _test_rig_pose_inv; // base link pos
    // wrt test rig link

    Eigen::VectorXd base_link_pos_est = _base_link_pos_rel_test_rig.translation(); // test rig joint position

    Eigen::VectorXd base_link_tvel_est = _base_link_vel; // for now use

    pssv_jnt_pos = base_link_pos_est(base_link_pos_est.size() - 1); // last element(z-component)

    pssv_jnt_vel = base_link_tvel_est(base_link_tvel_est.size() - 1);

    pssv_jnt_acc = (pssv_jnt_vel -
                    _q_p_dot_ft_est_prev(0) ) / _plugin_dt; // numerical diff

    }

}

void ContactEstRt::update_state_estimates()
{
    double passive_jnt_pos = 0, passive_jnt_vel = 0, pssv_jnt_acc = 0;

    get_passive_jnt_est(passive_jnt_pos, passive_jnt_vel, pssv_jnt_acc);

    _q_p_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_ft_est(0) = passive_jnt_pos; // assign passive dofs

    _q_p_dot_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_dot_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_dot_ft_est(0) = passive_jnt_vel; // assign passive dofs

//    _q_p_ddot_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = ( _q_p_dot_meas - _auxiliary_vector ) / _plugin_dt; // assign actuated dofs with meas.
    // from encoders
    _q_p_ddot_ft_est(0) = pssv_jnt_acc; // assign passive dofs

    _tau_ft_est.block(_n_jnts_model_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_meas; // assign actuated dofs with meas.
    // from encoders
    _tau_ft_est(0) = 0; // no effort on passive joints

}

void ContactEstRt::update_state()
{    
    _q_p_dot_ft_est_prev = _q_p_dot_ft_est; // getting estimated  joint
    // velocity before new state sensing

    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    get_fts_force();

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
//    _model_ft_est->setJointAcceleration(_q_p_ddot_ft_est); // update joint accelerations
    _model_ft_est->setJointEffort(_tau_ft_est); // update joint efforts

    _model_ft_est->update(); // update the model

//    _ft_estimator->update_estimate(); // we can now update the
    // force estimation

//    _f_cont_est = _ft_estimator->get_f(); // getting the estimated contact force

    // getting estimates of the tip and hip position
    // based on the reconstructed state used to update
    // the force estimation model
    _model_ft_est->getPose(_tip_link_name, _tip_pose_abs_est);
    _model_ft_est->getPose(_base_link_name, _base_link_abs_est);

}

void ContactEstRt::get_abs_tip_position()
{

    _tip_abs_position = _tip_pose_abs.translation();

}

void ContactEstRt::get_fts_force()
{
  if(_is_sim && _ft_tip_sensor_found)
  { // fts only available in simulation (for now)

    Eigen::Vector6d wrench = _ft_sensor->getWrench();

    _meas_tip_f_loc = wrench.head(3);
    _meas_tip_t_loc = wrench.tail(3);

  }

  // rotating the force into world frame
  // then from base to world orientation

  _meas_tip_f_abs = _tip_pose_abs * _meas_tip_f_loc;

}

void ContactEstRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/ContactEstRt", opt); // date-time automatically appended
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
    _dump_logger->add("is_sim", int(_is_sim));

    _dump_logger->create("q_p_meas", _n_jnts_robot), 1, _matlogger_buffer_size;
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("f_contact_ref", 3, 1, _matlogger_buffer_size);
    _dump_logger->create("f_contact_meas", 3, 1, _matlogger_buffer_size);
    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
      _dump_logger->create("tip_pos_meas", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("tip_pose_abs_est", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_abs_est", 3, 1, _matlogger_buffer_size);
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

}

void ContactEstRt::add_data2dump_logger()
{

    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

    _dump_logger->add("f_contact_ref", _f_contact_ref);

    _dump_logger->add("tip_pos_rel_base_link", _tip_pose_rel_base_link.translation());

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
        _dump_logger->add("tip_pos_meas", _tip_abs_position);

        _dump_logger->add("tip_pos_est", _tip_pose_abs_est.translation());

        _dump_logger->add("base_link_abs", _base_link_abs.translation());

        _dump_logger->add("base_link_abs_est", _base_link_abs_est.translation());

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

void ContactEstRt::init_nrt_ros_bridge()
{    
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Subscribers */
    _base_link_pose_sub = _ros->subscribe("/xbotcore/link_state/base_link/pose",
                                &ContactEstRt::on_base_link_pose_received,
                                this,
                                1,  // queue size
                                &_queue);

    _base_link_twist_sub = _ros->subscribe("/xbotcore/link_state/base_link/twist",
                                &ContactEstRt::on_base_link_twist_received,
                                this,
                                1,  // queue size
                                &_queue);

    /* Publishers */
//    awesome_leg::ContactEstStatus contact_st_prealloc;
//    replay_st_prealloc.approach_traj_finished = false;

//    _contact_est_pub = _ros->advertise<awesome_leg::MatReplayerStatus>(
//        "contact_est_node", 1, replay_st_prealloc);

}

void ContactEstRt::on_base_link_pose_received(const geometry_msgs::PoseStamped& msg)
{   

    tf::poseMsgToEigen(msg.pose, _base_link_abs);

    _tip_pose_abs = _base_link_abs * _tip_pose_rel_base_link; // from tip to base link orientation,

    get_abs_tip_position();
}

void ContactEstRt::on_base_link_twist_received(const geometry_msgs::TwistStamped& msg)
{

    Eigen::Vector6d twist;

    tf::twistMsgToEigen(msg.twist, twist);

    _base_link_vel = twist.head(3);
    _base_link_omega = twist.tail(3);
}

//void ContactEstRt::pub_contact_est_status()
//{
//    auto status_msg = _contact_est_pub->loanMessage();
//    status_msg->msg().approach_traj_finished = _approach_traj_finished;

//    _contact_est_pub->publishLoaned(std::move(status_msg));
//}

bool ContactEstRt::on_initialize()
{ 
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    // Initializing XBot2 model interface using the read parameters
    init_model_interface();

    _n_jnts_robot = _robot->getJointNum();

    init_vars();

    init_nrt_ros_bridge();

    create_ros_api();

    init_ft_sensor();

    init_ft_estimator();

    return true;
    
}

void ContactEstRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers
    
    update_state(); // read current jnt positions and velocities

    // Move on to run()
    start_completed();
    
}

void ContactEstRt::run()
{  
    int i = 0;

    update_state(); // update all necessary states

    _queue.run();

//    pub_contact_est_status();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

}

void ContactEstRt::on_stop()
{
    _is_first_run = true;

    reset_flags();

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void ContactEstRt::stopping()
{

    stop_completed();
}

void ContactEstRt::on_abort()
{

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void ContactEstRt::on_close()
{
    jinfo("Closing ContactEstRt");
}

XBOT2_REGISTER_PLUGIN(ContactEstRt, contact_est_rt)

