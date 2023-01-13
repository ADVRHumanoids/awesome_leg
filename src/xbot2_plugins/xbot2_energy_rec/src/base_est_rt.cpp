#include "base_est_rt.h"

#include <math.h>

void BaseEstRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void BaseEstRt::init_vars()
{

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_ref = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_ref = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_ff = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_stiff = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_damp = Eigen::VectorXd::Zero(_n_jnts_robot);

    _K_p = Eigen::MatrixXd::Zero(_n_jnts_robot, _n_jnts_robot);
    _K_d = Eigen::MatrixXd::Zero(_n_jnts_robot, _n_jnts_robot);

    _base_link_vel = Eigen::VectorXd::Zero(3);
    _base_link_omega = Eigen::VectorXd::Zero(3);

}

void BaseEstRt::reset_flags()
{

}

void BaseEstRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void BaseEstRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path_base_est = getParamOrThrow<std::string>("~urdf_path_base_est");
    _srdf_path_base_est = getParamOrThrow<std::string>("~srdf_path_base_est");

    _ik_problem_path = getParamOrThrow<std::string>("~ik_problem_path");

    _mat_path = getParamOrThrow<std::string>("~mat_path");
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _tip_link_name = getParamOrThrow<std::string>("~tip_link_name");
    _base_link_name = getParamOrThrow<std::string>("~base_link_name");
    _test_rig_linkname = getParamOrThrow<std::string>("~test_rig_linkname");

    _tip_fts_name = getParamOrThrow<std::string>("~tip_fts_name");
    _contact_linkname = getParamOrThrow<std::string>("~contact_linkname");

    _mov_avrg_cutoff_freq = getParamOrThrow<double>("~mov_avrg_cutoff_freq");
}

void BaseEstRt::is_sim(std::string sim_string = "sim")
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

void BaseEstRt::init_model_interfaces()
{

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## URDF for xbot2 model interface loaded @ {}\n", _urdf_path_base_est);
    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## SRDF for xbot2 model interface loaded @ {}\n \n", _urdf_path_base_est);

    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path_base_est);
    xbot_cfg.set_srdf_path(_srdf_path_base_est);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // Initializing XBot2 ModelInterface for the rt thread
    _base_est_model = XBot::ModelInterface::getModel(xbot_cfg);

}

void BaseEstRt::init_base_estimator()
{
    _be_options.dt = _plugin_dt;
    _be_options.log_enabled = false;
    _be_options.contact_release_thr = 10.0; // [N]
    _be_options.contact_attach_thr = 5.0;

    // load problem
    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n ## IK problem description for CartesIO loaded @ {}\n \n", _ik_problem_path);

    auto ik_problem_yaml = YAML::LoadFile(_ik_problem_path);

    // create estimator
    _est = std::make_unique<BaseEstimation>(_base_est_model, ik_problem_yaml, _be_options);

}

void BaseEstRt::init_ft_sensor()
{

  if(_is_sim)
  { // fts only available in simulation (for now)

      _ft_sensor = _robot->getDevices<Hal::ForceTorque>().get_device(_tip_fts_name);

      _ft_tip_sensor_found = (!_ft_sensor) ?  false : true; // flag to signal the
      // presence (or absence) of the tip force torque sensor (calling methods on a nullptr
      // causes a seg. fault)

  }

}

void BaseEstRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "base_est");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "base_est_rt");
}

void BaseEstRt::update_state()
{

    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);
    _robot->getJointEffort(_tau_meas);
    _robot->getPositionReference(_q_p_ref);
    _robot->getVelocityReference(_q_p_dot_ref);
    _robot->getEffortReference(_tau_ff);
    _robot->getStiffness(_meas_stiff);
    _robot->getDamping(_meas_damp);

    get_tau_cmd(); // computes the joint-level impedance control

    get_fts_force();

}

void BaseEstRt::get_fts_force()
{
  if(_is_sim && _ft_tip_sensor_found)
  { // fts only available in simulation (for now)

    _meas_w_loc = _ft_sensor->getWrench();

    _meas_tip_f_loc = _meas_w_loc.head(3);
    _meas_tip_t_loc = _meas_w_loc.tail(3);

    // rotating the force into world frame
    // then from base to world orientation

    _meas_tip_f_abs = _R_world_from_tip * _meas_tip_f_loc; // from tip local to world
    _meas_tip_t_abs = _R_world_from_tip * _meas_tip_t_loc; // from tip local to world

    _meas_w_abs.segment(0, 3) = _meas_tip_f_abs;
    _meas_w_abs.segment(3, 3) = _meas_tip_t_abs;

    // filtering measured wrench (in world frame)
    _ft_meas_filt.add_sample(_meas_w_abs);
    _ft_meas_filt.get(_meas_w_filt);

  }

}

void BaseEstRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression

    if (!_is_sim)
    {
        _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("test_") + _dump_mat_suffix, opt); // date-time automatically appended
    }
    else
    {
        _dump_logger = MatLogger2::MakeLogger(_mat_path + std::string("sim_") + _dump_mat_suffix, opt); // date-time automatically appended
    }

    _dump_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    _dump_logger->add("plugin_dt", _plugin_dt);
    _dump_logger->add("is_sim", int(_is_sim));

    _dump_logger->create("q_p_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_ref", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_ref", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_ff", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("tip_f_est_abs", _meas_tip_f_abs.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("tip_t_est_abs", _meas_tip_t_abs.size(), 1, _matlogger_buffer_size);

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)

      _dump_logger->create("R_world_from_tip", 3, 3, _matlogger_buffer_size);

      _dump_logger->create("meas_tip_f_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_t_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_t_abs", 3, 1, _matlogger_buffer_size);

      _dump_logger->create("meas_tip_w_filt", _meas_w_abs.size(), 1, _matlogger_buffer_size);

    }

}

void BaseEstRt::add_data2dump_logger()
{
    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);
    _dump_logger->add("q_p_ref", _q_p_ref);
    _dump_logger->add("q_p_dot_ref", _q_p_dot_ref);
    _dump_logger->add("tau_ff", _tau_ff);
    _dump_logger->add("tau_cmd", _tau_cmd);

    _dump_logger->add("tip_f_est_abs", _meas_tip_f_abs);
    _dump_logger->add("tip_t_est_abs", _meas_tip_t_abs);

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)

        _dump_logger->add("meas_tip_f_loc", _meas_tip_f_loc);
        _dump_logger->add("meas_tip_f_abs", _meas_tip_f_abs);
        _dump_logger->add("meas_tip_t_loc", _meas_tip_t_loc);
        _dump_logger->add("meas_tip_t_abs", _meas_tip_t_abs);

        _dump_logger->add("meas_tip_w_filt", _meas_w_abs);

    }

}

void BaseEstRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Subscribers */
    _base_link_pose_sub = _ros->subscribe("/xbotcore/link_state/base_link/pose",
                                &BaseEstRt::on_base_link_pose_received,
                                this,
                                1,  // queue size
                                &_queue);

    _base_link_twist_sub = _ros->subscribe("/xbotcore/link_state/base_link/twist",
                                &BaseEstRt::on_base_link_twist_received,
                                this,
                                1,  // queue size
                                &_queue);

    /* Publishers */

}

void BaseEstRt::on_base_link_pose_received(const geometry_msgs::PoseStamped& msg)
{

    tf::poseMsgToEigen(msg.pose, _M_world_from_base_link);

}

void BaseEstRt::on_base_link_twist_received(const geometry_msgs::TwistStamped& msg)
{

    Eigen::Vector6d twist;

    tf::twistMsgToEigen(msg.twist, twist);

    _base_link_vel = twist.head(3);
    _base_link_omega = twist.tail(3);
}

void BaseEstRt::get_tau_cmd()
{
    // assign stiffness and damping matrices
    for (int i = 0; i < _q_p_meas.size(); i++)
    {
        _K_p(i, i) = _meas_stiff(i);
        _K_d(i, i) = _meas_damp(i);
    }

    _tau_cmd = _tau_ff - _K_p * (_q_p_meas - _q_p_ref) - _K_d * (_q_p_dot_meas - _q_p_dot_ref);

}

bool BaseEstRt::on_initialize()
{
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    // Initializing XBot2 model interface using the read parameters
    init_model_interfaces();

    _n_jnts_robot = _robot->getJointNum();

    init_vars();

    init_nrt_ros_bridge();

    create_ros_api();

    init_ft_sensor();

    init_base_estimator();

    _ft_meas_filt = MovAvrgFilt(_meas_w_abs.size(), _plugin_dt, _mov_avrg_cutoff_freq);

    return true;

}

void BaseEstRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    update_state(); // read current jnt positions and velocities

    // Move on to run()
    start_completed();

}

void BaseEstRt::run()
{
    update_state(); // update all necessary states

    _queue.run();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void BaseEstRt::on_stop()
{

    reset_flags();

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void BaseEstRt::stopping()
{

    stop_completed();
}

void BaseEstRt::on_abort()
{

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void BaseEstRt::on_close()
{
    jinfo("Closing ContactEstRt");
}

XBOT2_REGISTER_PLUGIN(BaseEstRt, base_est_rt)

