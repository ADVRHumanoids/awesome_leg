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

    _q_p_be = Eigen::VectorXd::Zero(_nq_be);
    _q_p_dot_be = Eigen::VectorXd::Zero(_nv_be);
    _q_p_ddot_be = Eigen::VectorXd::Zero(_nv_be);
    _tau_be = Eigen::VectorXd::Zero(_nv_be);

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

    _use_ground_truth_gz = getParamOrThrow<bool>("~use_ground_truth_gz");
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
        "\n BaseEstRt: URDF for xbot2 model interface loaded @ {}\n", _urdf_path_base_est);
    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n BaseEstRt: SRDF for xbot2 model interface loaded @ {}\n \n", _srdf_path_base_est);

    XBot::ConfigOptions xbot_cfg;
    xbot_cfg.set_urdf_path(_urdf_path_base_est);
    xbot_cfg.set_srdf_path(_srdf_path_base_est);
    xbot_cfg.generate_jidmap();
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // Initializing XBot2 ModelInterface for the rt thread
    _base_est_model = XBot::ModelInterface::getModel(xbot_cfg);

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n BaseEstRt: XBot2 model interface initialized successfully \n \n");

    _pin_model_ptr.reset(new ModelInterface::Model(_urdf_path_base_est));
    _nq_be = _pin_model_ptr->get_nq();
    _nv_be = _pin_model_ptr->get_nv();

    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
        "\n BaseEstRt: Pinocchio-based model interface initialized successfully \n \n");

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
    _est->setFilterOmega(0.0);
    _est->setFilterDamping(0.0);
    _est->setFilterTs(getPeriodSec());

    _ft_est = _est->createVirtualFt(_tip_fts_name, {0, 1, 2}); // estimating only force

}

void BaseEstRt::init_transforms()
{
    // getting link poses from model
    _pin_model_ptr->get_frame_pose(_base_link_name,
                              _M_world_from_base_link_ref); // the passive joint dof is

    _M_world_from_base_link = _M_world_from_base_link_ref; // initialization

    // defined wrt its neutral position

    _pin_model_ptr->get_frame_pose(_test_rig_linkname,
                              _M_test_rig_from_world); // test rig pose

    _M_world_from_test_rig = _M_test_rig_from_world.inverse(); // from world to test rig
    // (it's constant, so we get it here)

    _pin_model_ptr->get_frame_pose(_tip_link_name,
                              _M_world_from_tip);
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

void BaseEstRt::get_robot_state()
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
}

void BaseEstRt::get_base_est(double& pssv_jnt_pos,
                                double& pssv_jnt_vel)
{

    if(_is_sim && _use_ground_truth_gz)
    { // for now, estimates only available in simulation

        _M_base_link_ref_from_base_link = _M_world_from_base_link_ref.inverse() * _M_world_from_base_link;
        // _M_world_from_base_link is automatically updated by the ros topic callback method

        pssv_jnt_pos = _M_base_link_ref_from_base_link.translation()[2];

        _base_link_vel_wrt_test_rig = _M_test_rig_from_world.rotation() * _base_link_vel; // pure rotation from world to test rig
        pssv_jnt_vel = _base_link_vel_wrt_test_rig[2]; // extracting vertical component (== prismatic joint velocity)

    }
    else
    { // we employ estimates

        /* Update estimate */
        Eigen::Affine3d base_pose;
        Eigen::Vector6d base_vel, raw_base_vel;
        if(!_est->update(base_pose, base_vel, raw_base_vel))
        {
            jerror("unable to solve");
            return;
        }

        pssv_jnt_pos = 0.0;

        pssv_jnt_vel = 0.0;
    }

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

void BaseEstRt::update_base_estimates()
{
    double passive_jnt_pos = 0, passive_jnt_vel = 0;

    get_base_est(passive_jnt_pos, passive_jnt_vel);

    _q_p_be.block(_nv_be - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_be(0) = passive_jnt_pos; // assign passive dofs

    _q_p_dot_be.block(_nv_be - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_dot_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_dot_be(0) = passive_jnt_vel; // assign passive dofs

    _num_diff_v.add_sample(_q_p_dot_be); // update differentiation
    _num_diff_v.dot(_q_p_ddot_be); // getting differentiated state acceleration

    _tau_be.block(_nv_be - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_meas; // measured torque
    _tau_be(0) = 0; // no effort on passive joints

}

void BaseEstRt::update_be_model()
{
    _base_est_model->setJointPosition(_q_p_be);
    _base_est_model->setJointVelocity(_q_p_dot_be);
    _base_est_model->setJointEffort(_tau_be);
//    _base_est_model->setStiffness();
//    _base_est_model->setDamping();

    _base_est_model->update();

}

void BaseEstRt::update_pin_model()
{
    _pin_model_ptr->set_q(_q_p_be);
    _pin_model_ptr->set_v(_q_p_dot_be);
    _pin_model_ptr->set_tau(_tau_be);

    _pin_model_ptr->update();
}

void BaseEstRt::update_states()
{

    get_robot_state(); // gets states from the robot

    get_tau_cmd(); // computes the joint-level impedance control

    update_base_estimates();

    update_be_model();
    update_pin_model();

    _pin_model_ptr->get_frame_pose(_tip_link_name,
                              _M_world_from_tip);
    _R_world_from_tip = _M_world_from_tip.rotation();

    get_fts_force(); // after update tip pose, we get the local force and
    // rotate it to the world

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

    _num_diff_v = NumDiff(_nv_be, _plugin_dt);

    _ft_meas_filt = MovAvrgFilt(_meas_w_abs.size(), _plugin_dt, _mov_avrg_cutoff_freq);

    // setting pin model q to neutral to get the reference base link
    // height wrt the passive dof is defined
    _pin_model_ptr->set_neutral();
    _pin_model_ptr->update();

    init_transforms(); // we get/initialize some useful link poses

    return true;

}

void BaseEstRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    _est->reset();

    update_states(); // read current jnt positions and velocities

    // Move on to run()
    start_completed();

}

void BaseEstRt::run()
{
    update_states(); // update all necessary states

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
