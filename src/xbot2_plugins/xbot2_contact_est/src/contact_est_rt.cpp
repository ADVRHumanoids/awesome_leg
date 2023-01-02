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
    _q_p_ref = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_ref = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_ff = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_cmd = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_stiff = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_damp = Eigen::VectorXd::Zero(_n_jnts_robot);

    _K_p = Eigen::MatrixXd::Zero(_n_jnts_robot, _n_jnts_robot);
    _K_d = Eigen::MatrixXd::Zero(_n_jnts_robot, _n_jnts_robot);

    _tip_abs_position = Eigen::VectorXd::Zero(3);

    // states and contact force used for force estimation
    _q_p_ft_est = Eigen::VectorXd::Zero(_nq_ft_est);
    _q_p_dot_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _q_p_ddot_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _tau_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _tau_c = Eigen::VectorXd::Zero(_nv_ft_est);
    _CT_v = Eigen::VectorXd::Zero(_nv_ft_est);
    _g = Eigen::VectorXd::Zero(_nv_ft_est);
    _tau_c_raw = Eigen::VectorXd::Zero(_nv_ft_est);
    _p = Eigen::VectorXd::Zero(_nv_ft_est);
    _p_dot = Eigen::VectorXd::Zero(_nv_ft_est);
    _C = Eigen::MatrixXd::Zero(_nv_ft_est, _nv_ft_est);
    _v = Eigen::MatrixXd::Zero(_nv_ft_est, _nv_ft_est);
    _q = Eigen::MatrixXd::Zero(_nv_ft_est, _nv_ft_est);
    _tau = Eigen::MatrixXd::Zero(_nv_ft_est, _nv_ft_est);

    _meas_tip_f_abs = Eigen::VectorXd::Zero(3);
    _tip_f_est_abs = Eigen::VectorXd::Zero(3);
    _meas_tip_t_abs = Eigen::VectorXd::Zero(3);
    _tip_t_est_abs = Eigen::VectorXd::Zero(3);
    _base_link_pos = Eigen::VectorXd::Zero(3);
    _base_link_vel = Eigen::VectorXd::Zero(3);
    _base_link_omega = Eigen::VectorXd::Zero(3);

    _meas_tip_f_loc = Eigen::VectorXd::Zero(3);
    _meas_tip_t_loc = Eigen::VectorXd::Zero(3);

    _w_c_est = Eigen::VectorXd::Zero(6);

    // used to convert to ros messages-compatible types
    _tau_c_vect = std::vector<double>(_nv_ft_est);

    _tau_c_raw_vect = std::vector<double>(_nv_ft_est);
    _CT_v_vect = std::vector<double>(_nv_ft_est);
    _g_vect = std::vector<double>(_nv_ft_est);
    _p_vect = std::vector<double>(_nv_ft_est);
    _p_dot_vect = std::vector<double>(_nv_ft_est);
    _q_p_ft_est_vect = std::vector<double>(_nq_ft_est);
    _q_p_dot_ft_est_vect = std::vector<double>(_nv_ft_est);
    _tau_cmd_vect = std::vector<double>(_nv_ft_est);
    _i_c_vect = std::vector<double>(_nv_ft_est);
    _w_c_est_vect = std::vector<double>(6);
    _tip_f_est_abs_vect = std::vector<double>(3);
    _tip_t_est_abs_vect = std::vector<double>(3);
    _f_meas_vect = std::vector<double>(3);
    _w_meas_vect = std::vector<double>(3);
    _meas_tip_f_abs_vect = std::vector<double>(3);
    _meas_tip_t_abs_vect = std::vector<double>(3);
    _meas_tip_f_abs_filt_vect= std::vector<double>(3);
    _meas_tip_t_abs_filt_vect = std::vector<double>(3);

    if(!_estimate_full_wrench)
    {
        _selector = std::vector<int>{0, 1, 2};
    }

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

    _urdf_path_ft_est = getParamOrThrow<std::string>("~urdf_path_ft_est");

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

    _ft_est_lambda = getParamOrThrow<Eigen::VectorXd>("~ft_est_lambda");
    _select_est_bw_manually = getParamOrThrow<bool>("~select_est_bw_manually");

    _ft_est_bw = getParamOrThrow<double>("~ft_est_bw");
    _contacts.push_back(_contact_linkname);

    _estimate_full_wrench = getParamOrThrow<bool>("~estimate_full_wrench");

    _ft_meas_cutoff_freq = getParamOrThrow<double>("~meas_w_filt_bw");

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

void ContactEstRt::init_model_interfaces()
{

    _ft_est_model_ptr.reset(new Model(_urdf_path_ft_est));
    _nq_ft_est = _ft_est_model_ptr->get_nq();
    _nv_ft_est = _ft_est_model_ptr->get_nv();

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
    double bw = (_select_est_bw_manually) ? _ft_est_bw :  1/_plugin_dt;


    _ft_estimator.reset(new MomentumBasedFObs(_ft_est_model_ptr,
                                              _plugin_dt,
                                              _contacts,
                                              bw,
                                              _ft_est_lambda,
                                              true,
                                              _selector));

}

void ContactEstRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "mr");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "mat_replayer_rt");
}

void ContactEstRt::get_passive_jnt_est(double& pssv_jnt_pos,
                                        double& pssv_jnt_vel)
{

    if(_is_sim)
    { // for now, estimates only available in simulation

    _M_base_link_ref_from_base_link = _M_world_from_base_link_ref.inverse() * _M_world_from_base_link;

    pssv_jnt_pos = _M_base_link_ref_from_base_link.translation()[2];

    _base_link_vel_wrt_test_rig = _M_test_rig_from_world.rotation() * _base_link_vel; // pure rotation from world to test rig
    pssv_jnt_vel = _base_link_vel_wrt_test_rig[2]; // extracting vertical component (== prismatic joint velocity)

    }

}

void ContactEstRt::update_state_estimates()
{
    double passive_jnt_pos = 0, passive_jnt_vel = 0;

    get_passive_jnt_est(passive_jnt_pos, passive_jnt_vel);

    _q_p_ft_est.block(_nv_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_ft_est(0) = passive_jnt_pos; // assign passive dofs

    _q_p_dot_ft_est.block(_nv_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _q_p_dot_meas; // assign actuated dofs with meas.
    // from encoders
    _q_p_dot_ft_est(0) = passive_jnt_vel; // assign passive dofs

    _num_diff_v.add_sample(_q_p_dot_ft_est); // update differentiation
    _num_diff_v.dot(_q_p_ddot_ft_est); // getting differentiated state acceleration

    _tau_ft_est.block(_nv_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_meas; // measured torque
//    _tau_ft_est.block(_nv_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_cmd; // command torque
    _tau_ft_est(0) = 0; // no effort on passive joints

}

void ContactEstRt::update_state()
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
    // torque (used by the ft estimator)

    update_state_estimates(); // updates state estimates
    // (q, q_dot, q_ddot) and efforts

    _ft_est_model_ptr->set_q(_q_p_ft_est);
    _ft_est_model_ptr->set_v(_q_p_dot_ft_est);
    _ft_est_model_ptr->set_a(_q_p_ddot_ft_est);
    _ft_est_model_ptr->set_tau(_tau_ft_est);

    _ft_est_model_ptr->update();

    // getting link poses from model
    _ft_est_model_ptr->get_frame_pose(_tip_link_name,
                              _M_world_from_tip);
    _ft_est_model_ptr->get_frame_pose(_base_link_name,
                              _M_world_from_base_link);

    _R_world_from_tip = _M_world_from_tip.rotation();

    get_fts_force(); // after update tip pose, we get the local force and
    // rotate it to the world

    // getting other quantities which are useful for debugging the estimator
    _ft_est_model_ptr->get_C(_C);
    _ft_est_model_ptr->get_v(_v);
    _ft_est_model_ptr->get_g(_g);
    _ft_est_model_ptr->get_p(_p);
    _ft_est_model_ptr->get_tau(_tau);

    _num_diff_p.add_sample(_p); // differentiating the generalized momentum
    _num_diff_p.dot(_p_dot);
    _CT_v = _C.transpose() * _v;
    _tau_c_raw = _p_dot - _CT_v + _g - _tau; // raw disturbance torques (not filtered
    // and without observer)

    _ft_estimator->update(); // we can now update the
    // force estimation
    _ft_estimator->get_tau_obs(_tau_c);
    _ft_estimator->get_w_est_at(_contacts[0], _w_c_est);
    _ft_estimator->get_f_est_at(_contacts[0], _tip_f_est_abs);
    _ft_estimator->get_t_est_at(_contacts[0], _tip_t_est_abs);

    _i_c = _w_c_est * _plugin_dt;

}

void ContactEstRt::get_fts_force()
{
  if(_is_sim && _ft_tip_sensor_found)
  { // fts only available in simulation (for now)

    _meas_w_loc = _ft_sensor->getWrench();

    _meas_tip_f_loc = _meas_w_loc.head(3);
    _meas_tip_t_loc = _meas_w_loc.tail(3);

  }

  // rotating the force into world frame
  // then from base to world orientation

  _meas_tip_f_abs = _R_world_from_tip * _meas_tip_f_loc; // from tip local to world
  _meas_tip_t_abs = _R_world_from_tip * _meas_tip_t_loc; // from tip local to world

  _meas_w_abs.segment(0, 3) = _meas_tip_f_abs;
  _meas_w_abs.segment(3, 3) = _meas_tip_t_abs;

  // filtering measured wrench (in world frame)
  _ft_meas_filt.add_sample(_meas_w_abs);
  _ft_meas_filt.get(_meas_w_filt);

  _meas_tip_f_abs_filt = _meas_w_filt.head(3);

  _meas_tip_t_abs_filt = _meas_w_filt.tail(3);

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

    _dump_logger->create("q_p_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_ref", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("q_p_dot_ref", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_ff", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_cmd", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("i_c", 6, 1, _matlogger_buffer_size);

    _dump_logger->create("tip_f_est_abs", _meas_tip_f_abs.size(), 1, _matlogger_buffer_size);
    _dump_logger->create("tip_t_est_abs", _meas_tip_f_abs.size(), 1, _matlogger_buffer_size);

    _dump_logger->create("C", _nv_ft_est, _nv_ft_est, _matlogger_buffer_size);
    _dump_logger->create("R_world_from_tip", 3, 3, _matlogger_buffer_size);

    _dump_logger->create("g", _nv_ft_est, 1, _matlogger_buffer_size);
    _dump_logger->create("p", _nv_ft_est, 1, _matlogger_buffer_size);
    _dump_logger->create("p_dot", _nv_ft_est, 1, _matlogger_buffer_size);
    _dump_logger->create("CT_v", _nv_ft_est, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_c_raw", _nv_ft_est, 1, _matlogger_buffer_size);

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
      _dump_logger->create("base_link_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_abs_est", 3, 1, _matlogger_buffer_size);

      _dump_logger->create("meas_tip_f_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_t_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_t_abs", 3, 1, _matlogger_buffer_size);

      _dump_logger->create("meas_tip_f_abs_filt", _meas_tip_f_abs.size(), 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_t_abs_filt", _meas_tip_f_abs.size(), 1, _matlogger_buffer_size);

      _dump_logger->create("base_link_vel", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_omega", 3, 1, _matlogger_buffer_size);

      _dump_logger->create("q_p_ft_est", _nq_ft_est), 1, _matlogger_buffer_size;
      _dump_logger->create("q_p_dot_ft_est", _nv_ft_est, 1, _matlogger_buffer_size);
      _dump_logger->create("q_p_ddot_ft_est", _nv_ft_est, 1, _matlogger_buffer_size);
      _dump_logger->create("tau_ft_est", _nv_ft_est, 1, _matlogger_buffer_size);
    }

}

void ContactEstRt::add_data2dump_logger()
{
    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);
    _dump_logger->add("q_p_ref", _q_p_ref);
    _dump_logger->add("q_p_dot_ref", _q_p_dot_ref);
    _dump_logger->add("tau_ff", _tau_ff);
    _dump_logger->add("tau_cmd", _tau_cmd);

    _dump_logger->add("C", _C);
    _dump_logger->add("R_world_from_tip", _R_world_from_tip);
    _dump_logger->add("g", _g);
    _dump_logger->add("p", _p);
    _dump_logger->add("p_dot", _p_dot);
    _dump_logger->add("CT_v", _CT_v);
    _dump_logger->add("tau_c_raw", _tau_c_raw);
    _dump_logger->add("tau_c_raw", _tau_c_raw);

    _dump_logger->add("tip_f_est_abs", _tip_f_est_abs);
    _dump_logger->add("tip_t_est_abs", _tip_t_est_abs);

    _dump_logger->add("i_c", _i_c);

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
        _dump_logger->add("base_link_abs", _M_world_from_base_link.translation());
        _dump_logger->add("base_link_abs_est", _M_world_from_base_link.translation());

        _dump_logger->add("meas_tip_f_loc", _meas_tip_f_loc);
        _dump_logger->add("meas_tip_f_abs", _meas_tip_f_abs);
        _dump_logger->add("meas_tip_t_loc", _meas_tip_t_loc);
        _dump_logger->add("meas_tip_t_abs", _meas_tip_t_abs);

        _dump_logger->add("meas_tip_f_abs_filt", _meas_tip_f_abs_filt);
        _dump_logger->add("meas_tip_t_abs_filt", _meas_tip_t_abs_filt);

        _dump_logger->add("base_link_vel", _base_link_vel);
        _dump_logger->add("base_link_omega", _base_link_omega);

        _dump_logger->add("q_p_ft_est", _q_p_ft_est);
        _dump_logger->add("q_p_dot_ft_est", _q_p_dot_ft_est);
        _dump_logger->add("q_p_ddot_ft_est", _q_p_ddot_ft_est);
        _dump_logger->add("tau_ft_est", _tau_ft_est);

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
    awesome_leg::ContactEstStatus contact_est_prealloc;

    std::vector<double> tau_c_prealloc(_nv_ft_est);

    std::vector<double> tau_c_raw_prealloc(_nv_ft_est);
    std::vector<double> CT_v_prealloc(_nv_ft_est);
    std::vector<double> g_prealloc(_nv_ft_est);
    std::vector<double> p_prealloc(_nv_ft_est);
    std::vector<double> p_dot_prealloc(_nv_ft_est);
    std::vector<double> q_p_ft_est_prealloc(_nv_ft_est);
    std::vector<double> q_p_dot_ft_est_prealloc(_nv_ft_est);
    std::vector<double> tau_cmd_prealloc(_nv_ft_est);
    std::vector<double> i_c_prealloc(6);

    std::vector<double> f_c_prealloc(3);
    std::vector<double> t_c_prealloc(3);
    std::vector<double> f_meas_prealloc(3);
    std::vector<double> t_meas_prealloc(3);
    std::vector<double> f_meas_filt_prealloc(3);
    std::vector<double> t_meas_filt_prealloc(3);

    contact_est_prealloc.tau_c = tau_c_prealloc;
    contact_est_prealloc.tau_cmd = tau_cmd_prealloc;

    contact_est_prealloc.i_c = i_c_prealloc;

    contact_est_prealloc.tau_c_raw = tau_c_raw_prealloc;
    contact_est_prealloc.g = g_prealloc;
    contact_est_prealloc.p = p_prealloc;
    contact_est_prealloc.p_dot = p_dot_prealloc;
    contact_est_prealloc.CT_v = CT_v_prealloc;
    contact_est_prealloc.q = q_p_ft_est_prealloc;
    contact_est_prealloc.v = q_p_dot_ft_est_prealloc;

    contact_est_prealloc.f_c = f_c_prealloc;
    contact_est_prealloc.t_c = t_c_prealloc;

    contact_est_prealloc.f_meas = f_meas_prealloc;
    contact_est_prealloc.t_meas = t_meas_prealloc;
    contact_est_prealloc.f_meas_filt = f_meas_filt_prealloc;
    contact_est_prealloc.t_meas_filt = t_meas_filt_prealloc;

    _cont_est_status_pub = _ros->advertise<awesome_leg::ContactEstStatus>(
        "contact_est_node", 1, contact_est_prealloc);

}

void ContactEstRt::get_tau_cmd()
{
    // assign stiffness and damping matrices
    for (int i = 0; i < _q_p_meas.size(); i++)
    {
        _K_p(i, i) = _meas_stiff(i);
        _K_d(i, i) = _meas_damp(i);
    }

    _tau_cmd = _tau_ff - _K_p * (_q_p_meas - _q_p_ref) - _K_d * (_q_p_dot_meas - _q_p_dot_ref);

}

void ContactEstRt::on_base_link_pose_received(const geometry_msgs::PoseStamped& msg)
{   

    tf::poseMsgToEigen(msg.pose, _M_world_from_base_link);

}

void ContactEstRt::on_base_link_twist_received(const geometry_msgs::TwistStamped& msg)
{

    Eigen::Vector6d twist;

    tf::twistMsgToEigen(msg.twist, twist);

    _base_link_vel = twist.head(3);
    _base_link_omega = twist.tail(3);
}

void ContactEstRt::pub_contact_est_status()
{
    auto status_msg = _cont_est_status_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published
    Eigen::Map<Eigen::VectorXd>(&_tau_c_vect[0], _tau_c.size(), 1) = _tau_c;

    Eigen::Map<Eigen::VectorXd>(&_tau_cmd_vect[0], _tau.size(), 1) = _tau;
    Eigen::Map<Eigen::VectorXd>(&_tau_c_raw_vect[0], _tau_c_raw.size(), 1) = _tau_c_raw;
    Eigen::Map<Eigen::VectorXd>(&_g_vect[0], _g.size(), 1) = _g;
    Eigen::Map<Eigen::VectorXd>(&_p_vect[0], _p.size(), 1) = _p;
    Eigen::Map<Eigen::VectorXd>(&_p_dot_vect[0], _p_dot.size(), 1) = _p_dot;
    Eigen::Map<Eigen::VectorXd>(&_CT_v_vect[0], _CT_v.size(), 1) = _CT_v;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ft_est_vect[0], _q.size(), 1) = _q;
    Eigen::Map<Eigen::VectorXd>(&_q_p_dot_ft_est_vect[0], _v.size(), 1) = _v;

    Eigen::Map<utils_defs::Wrench>(&_w_c_est_vect[0], _w_c_est.size(), 1) = _w_c_est;
    Eigen::Map<utils_defs::Wrench>(&_w_c_est_vect[0], _w_c_est.size(), 1) = _w_c_est;
    Eigen::Map<utils_defs::Force3D>(&_tip_f_est_abs_vect[0], _tip_f_est_abs.size(), 1) = _tip_f_est_abs;
    Eigen::Map<utils_defs::Torque3D>(&_tip_t_est_abs_vect[0], _tip_t_est_abs.size(), 1) = _tip_t_est_abs;
    Eigen::Map<utils_defs::Force3D>(&_meas_tip_f_abs_vect[0], _meas_tip_f_abs.size(), 1) = _meas_tip_f_abs;
    Eigen::Map<utils_defs::Torque3D>(&_meas_tip_t_abs_vect[0], _meas_tip_t_abs.size(), 1) = _meas_tip_t_abs;
    Eigen::Map<utils_defs::Force3D>(&_meas_tip_f_abs_filt_vect[0], _meas_tip_f_abs_filt.size(), 1) = _meas_tip_f_abs_filt;
    Eigen::Map<utils_defs::Torque3D>(&_meas_tip_t_abs_filt_vect[0], _meas_tip_t_abs_filt.size(), 1) = _meas_tip_t_abs_filt;
    Eigen::Map<utils_defs::Wrench>(&_i_c_vect[0], _i_c.size(), 1) = _i_c;

    // filling message

    status_msg->msg().tau_c = _tau_c_vect;
    status_msg->msg().w_c = _w_c_est_vect;
    status_msg->msg().f_c = _tip_f_est_abs_vect;
    status_msg->msg().t_c = _tip_t_est_abs_vect;

    status_msg->msg().tau_c_raw = _tau_c_raw_vect;
    status_msg->msg().tau_cmd = _tau_cmd_vect;

    status_msg->msg().i_c = _i_c_vect;

    status_msg->msg().g = _g_vect;
    status_msg->msg().p = _p_vect;
    status_msg->msg().p_dot = _p_dot_vect;
    status_msg->msg().CT_v = _CT_v_vect;
    status_msg->msg().q = _q_p_ft_est_vect;
    status_msg->msg().v = _q_p_dot_ft_est_vect;

    status_msg->msg().f_meas = _meas_tip_f_abs_vect;
    status_msg->msg().t_meas = _meas_tip_t_abs_vect;
    status_msg->msg().f_meas_filt = _meas_tip_f_abs_filt_vect;
    status_msg->msg().t_meas_filt = _meas_tip_t_abs_filt_vect;

    status_msg->msg().contact_frames = _contacts;

    _cont_est_status_pub->publishLoaned(std::move(status_msg));
}

bool ContactEstRt::on_initialize()
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

    init_ft_estimator();

    _num_diff_v = NumDiff(_nv_ft_est, _plugin_dt);
    _num_diff_p = NumDiff(_nv_ft_est, _plugin_dt);

    _ft_meas_filt = MovAvrgFilt(_w_c_est.size(), _plugin_dt, _ft_meas_cutoff_freq);

    // setting model position to zero to get the reference base link
    // height wrt the passive dof is defined
    _ft_est_model_ptr->set_q(Eigen::VectorXd::Zero(_nq_ft_est));
    _ft_est_model_ptr->update();

    // getting link poses from model
    _ft_est_model_ptr->get_frame_pose(_base_link_name,
                              _M_world_from_base_link_ref);

    _ft_est_model_ptr->get_frame_pose(_test_rig_linkname,
                              _M_test_rig_from_world);
    _M_world_from_test_rig = _M_test_rig_from_world.inverse(); // from world to test rig

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

    pub_contact_est_status();

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

