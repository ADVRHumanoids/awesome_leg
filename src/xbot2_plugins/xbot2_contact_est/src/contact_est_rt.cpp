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

    _tip_abs_position = Eigen::VectorXd::Zero(3);

    // states and contact force used for force estimation
    _q_p_ft_est = Eigen::VectorXd::Zero(_nq_ft_est);
    _q_p_dot_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _q_p_ddot_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _tau_ft_est = Eigen::VectorXd::Zero(_nv_ft_est);
    _tau_c = Eigen::VectorXd::Zero(_nv_ft_est);

    _meas_tip_f_abs = Eigen::VectorXd::Zero(3);
    _tip_f_est_abs = Eigen::VectorXd::Zero(3);
    _meas_tip_t_abs = Eigen::VectorXd::Zero(3);
    _tip_t_est_abs = Eigen::VectorXd::Zero(3);
    _base_link_pos = Eigen::VectorXd::Zero(3);
    _base_link_vel = Eigen::VectorXd::Zero(3);
    _base_link_omega = Eigen::VectorXd::Zero(3);

    _meas_tip_f_loc = Eigen::VectorXd::Zero(3);
    _meas_tip_t_loc = Eigen::VectorXd::Zero(3);

    // used to convert to ros messages-compatible types
    _tau_c_vect = std::vector<double>(_nv_ft_est);
    _w_c_vect = std::vector<double>(6);
    _tip_f_est_abs_vect = std::vector<double>(3);
    _tip_t_est_abs_vect = std::vector<double>(3);
    _f_meas_vect = std::vector<double>(3);
    _w_meas_vect = std::vector<double>(3);
    _meas_tip_f_abs_vect = std::vector<double>(3);
    _meas_tip_t_abs_vect = std::vector<double>(3);

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

    _ft_est_lambda = getParamOrThrow<double>("~ft_est_lambda");
    _ft_est_bw = getParamOrThrow<double>("~ft_est_bw");

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

    _ft_estimator.reset(new MomentumBasedFObs(_ft_est_model_ptr, _plugin_dt, _ft_est_bw, _ft_est_lambda, true));

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
//    _base_link_pos_rel_test_rig = _base_link_abs * _test_rig_pose_inv; // base link pos
    // wrt test rig link

//    Eigen::VectorXd base_link_pos_est = _base_link_pos_rel_test_rig.translation(); // test rig joint position

    Eigen::VectorXd base_link_vel_est = _base_link_vel;

    pssv_jnt_pos = 0; // position doen't matter for now

    pssv_jnt_vel = base_link_vel_est(base_link_vel_est.size() - 1);

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

    _num_diff.add_sample(_q_p_dot_ft_est); // update differentiation
    _num_diff.dot(_q_p_ddot_ft_est); // getting differentiated state acceleration

    _tau_ft_est.block(_nv_ft_est - _n_jnts_robot, 0, _n_jnts_robot, 1) = _tau_meas; // assign actuated dofs with meas.
    // from encoders
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

    get_fts_force();

    update_state_estimates(); // updates state estimates
    // (q, q_dot, q_ddot) and efforts

    _ft_est_model_ptr->update(_q_p_ft_est, _q_p_dot_ft_est, _tau_ft_est, _q_p_ddot_ft_est);

    _ft_estimator->update(_contact_linkname); // we can now update the
    // force estimation
    _ft_estimator->get_tau_obs(_tau_c);
    _ft_estimator->get_w_est(_w_c);
    _ft_estimator->get_f_est(_tip_f_est_abs);
    _ft_estimator->get_t_est(_tip_t_est_abs);

    // getting estimates of the tip and hip position
    // based on the reconstructed state used to update
    // the force estimation model
//    _ft_est_model_ptr->getPose(_tip_link_name, _tip_pose_abs_est);
//    _ft_est_model_ptr->getPose(_base_link_name, _base_link_abs_est);

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

//  _meas_tip_f_abs = _tip_pose_abs * _meas_tip_f_loc;

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


    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
      _dump_logger->create("base_link_abs", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("base_link_abs_est", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_loc", 3, 1, _matlogger_buffer_size);
      _dump_logger->create("meas_tip_f_abs", 3, 1, _matlogger_buffer_size);
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

    if (_is_sim)
    { // no estimate of base link abs position on the real robot (for now)
        _dump_logger->add("base_link_abs", _base_link_abs.translation());
        _dump_logger->add("base_link_abs_est", _base_link_abs_est.translation());
        _dump_logger->add("meas_tip_f_loc", _meas_tip_f_loc);
        _dump_logger->add("meas_tip_f_abs", _meas_tip_f_abs);
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
    std::vector<double> f_c_prealloc(3);
    std::vector<double> t_c_prealloc(3);
    std::vector<double> f_meas_prealloc(3);
    std::vector<double> t_meas_prealloc(3);

    contact_est_prealloc.tau_c = tau_c_prealloc;
    contact_est_prealloc.f_c = f_c_prealloc;
    contact_est_prealloc.t_c = t_c_prealloc;

    contact_est_prealloc.f_meas = f_meas_prealloc;
    contact_est_prealloc.t_meas = t_meas_prealloc;

    _cont_est_status_pub = _ros->advertise<awesome_leg::ContactEstStatus>(
        "contact_est_node", 1, contact_est_prealloc);

}

void ContactEstRt::on_base_link_pose_received(const geometry_msgs::PoseStamped& msg)
{   

    tf::poseMsgToEigen(msg.pose, _base_link_abs);

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
    Eigen::Map<Eigen::VectorXd>(&_w_c_vect[0], _w_c.size(), 1) = _w_c;
    Eigen::Map<Eigen::VectorXd>(&_tip_f_est_abs_vect[0], _tip_f_est_abs.size(), 1) = _tip_f_est_abs;
    Eigen::Map<Eigen::VectorXd>(&_tip_t_est_abs_vect[0], _tip_t_est_abs.size(), 1) = _tip_t_est_abs;
    Eigen::Map<Eigen::VectorXd>(&_meas_tip_f_abs_vect[0], _meas_tip_f_abs.size(), 1) = _meas_tip_f_abs;
    Eigen::Map<Eigen::VectorXd>(&_meas_tip_t_abs_vect[0], _meas_tip_t_abs.size(), 1) = _meas_tip_t_abs;

    // filling message

    status_msg->msg().tau_c = _tau_c_vect;
    status_msg->msg().w_c = _w_c_vect;
    status_msg->msg().f_c = _tip_f_est_abs_vect;
    status_msg->msg().t_c = _tip_t_est_abs_vect;

    status_msg->msg().f_meas = _meas_tip_f_abs_vect;
    status_msg->msg().t_meas = _meas_tip_t_abs_vect;

    status_msg->msg().contact_frame = _contact_linkname;

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

    _num_diff = NumDiff(_nv_ft_est, _plugin_dt);

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

