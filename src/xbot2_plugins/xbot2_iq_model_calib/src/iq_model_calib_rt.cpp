#include "iq_model_calib_rt.h"

#include <math.h> 

void IqModelCalibRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void IqModelCalibRt::init_vars()
{

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_ddot_est = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_est = Eigen::VectorXd::Zero(_n_jnts_robot);
    _iq_jnt_names = std::vector<std::string>(_n_jnts_robot);

    _tau_req = Eigen::VectorXd::Zero(_n_jnts_robot);

    // used to convert to ros messages-compatible types
    _iq_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_meas_vect = std::vector<double>(_n_jnts_robot);
    _tau_meas_vect = std::vector<double>(_n_jnts_robot);
    _K_t_vect = std::vector<double>(_n_jnts_robot);
    _K_d0_vect = std::vector<double>(_n_jnts_robot);
    _K_d1_vect = std::vector<double>(_n_jnts_robot);
    _rot_MoI_vect = std::vector<double>(_n_jnts_robot);
    _red_ratio_vect = std::vector<double>(_n_jnts_robot);
    _tau_req_vect = std::vector<double>(_n_jnts_robot);

}

void IqModelCalibRt::reset_flags()
{

    _sample_index = 0.0; // resetting samples index, in case the plugin stopped and started again


}

void IqModelCalibRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }
    
}

void IqModelCalibRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _urdf_path = getParamOrThrow<std::string>("~urdf_path"); 
    _srdf_path = getParamOrThrow<std::string>("~srdf_path"); 

    _mat_path = getParamOrThrow<std::string>("~mat_path"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _verbose = getParamOrThrow<bool>("~verbose");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");
    _K_t = getParamOrThrow<Eigen::VectorXd>("~K_t");
    _K_d0 = getParamOrThrow<Eigen::VectorXd>("~K_d0");
    _K_d1 = getParamOrThrow<Eigen::VectorXd>("~K_d1");
    _rot_MoI = getParamOrThrow<Eigen::VectorXd>("~rotor_axial_MoI");

    _der_est_order = getParamOrThrow<int>("~der_est_order");

}

void IqModelCalibRt::is_sim(std::string sim_string = "sim")
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

void IqModelCalibRt::update_state()
{    
    // "sensing" the robot
    _robot->sense();

    // Getting robot state
    _robot->getJointPosition(_q_p_meas);
    _robot->getMotorVelocity(_q_p_dot_meas);  
    _robot->getJointEffort(_tau_meas);

    // Getting q_p_ddot via numerical differentiation
    _num_diff.add_sample(_q_p_dot_meas);
    _num_diff.dot(_q_p_ddot_est);

}

void IqModelCalibRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/IqModelCalibRt", opt); // date-time automatically appended
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

    _dump_logger->create("q_p_ddot_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->add("der_est_order", _der_est_order);

    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("iq_est", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("K_t", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d0", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio", _n_jnts_robot, 1, _matlogger_buffer_size);

}

void IqModelCalibRt::add_data2dump_logger()
{

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("q_p_ddot_est", _q_p_ddot_est);
    _dump_logger->add("tau_meas", _tau_meas);

    _dump_logger->add("iq_est", _iq_est);

    _dump_logger->add("K_t", _K_t);
    _dump_logger->add("K_d0", _K_d0);
    _dump_logger->add("K_d1", _K_d1);
    _dump_logger->add("rot_MoI", _rot_MoI);
    _dump_logger->add("red_ratio", _red_ratio);

}

void IqModelCalibRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Subscribers */
    _aux_signals_sub = _ros->subscribe("/xbotcore/aux",
                                &CalibUtils::IqRosGetter::on_aux_signal_received,
                                &_iq_getter,
                                1,  // queue size
                                &_queue);

    _js_signals_sub = _ros->subscribe("/xbotcore/joint_states",
                                &CalibUtils::IqRosGetter::on_js_signal_received,
                                &_iq_getter,
                                1,  // queue size
                                &_queue);


    /* Publishers */
    awesome_leg::IqEstStatus iq_status_prealloc;

    std::vector<double> iq_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_ddot_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_dot_meas_prealloc(_n_jnts_robot);
    std::vector<double> tau_meas_prealloc(_n_jnts_robot);
    std::vector<double> Kt_prealloc(_n_jnts_robot);
    std::vector<double> Kd0_prealloc(_n_jnts_robot);
    std::vector<double> Kd1_prealloc(_n_jnts_robot);
    std::vector<double> rot_MoI_prealloc(_n_jnts_robot);
    std::vector<double> red_ratio_prealloc(_n_jnts_robot);
    std::vector<double> tau_req_prealloc(_n_jnts_robot);

    std::vector<std::string> iq_jnt_names_prealloc(_n_jnts_robot);

    iq_status_prealloc.iq_est = iq_est_prealloc;
    iq_status_prealloc.q_p_ddot_est = q_p_ddot_est_prealloc;
    iq_status_prealloc.q_p_dot_meas = q_p_dot_meas_prealloc;
    iq_status_prealloc.tau_meas = tau_meas_prealloc;
    iq_status_prealloc.K_t = Kt_prealloc;
    iq_status_prealloc.K_d0 = Kd0_prealloc;
    iq_status_prealloc.K_d1 = Kd1_prealloc;
    iq_status_prealloc.rot_MoI = rot_MoI_prealloc;
    iq_status_prealloc.red_ratio = red_ratio_prealloc;
    iq_status_prealloc.tau_req = tau_req_prealloc;
    iq_status_prealloc.der_est_order = _der_est_order;

    iq_status_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    _iq_est_pub = _ros->advertise<awesome_leg::IqEstStatus>(
        "iq_est_node", 1, iq_status_prealloc);

}

void IqModelCalibRt::add_data2bedumped()
{

    add_data2dump_logger();
    
}

void IqModelCalibRt::pub_iq_est()
{
    auto iq_est_msg = _iq_est_pub->loanMessage();

    Eigen::Map<Eigen::VectorXd>(&_iq_est_vect[0], _iq_est.size(), 1) = _iq_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ddot_est_vect[0], _q_p_ddot_est.size(), 1) = _q_p_ddot_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_dot_meas_vect[0], _q_p_dot_meas.size(), 1) = _q_p_dot_meas;
    Eigen::Map<Eigen::VectorXd>(&_tau_meas_vect[0], _tau_meas.size(), 1) = _tau_meas;
    Eigen::Map<Eigen::VectorXd>(&_K_t_vect[0], _K_t.size(), 1) = _K_t;
    Eigen::Map<Eigen::VectorXd>(&_K_d0_vect[0], _K_d0.size(), 1) = _K_d0;
    Eigen::Map<Eigen::VectorXd>(&_K_d1_vect[0], _K_d1.size(), 1) = _K_d1;
    Eigen::Map<Eigen::VectorXd>(&_rot_MoI_vect[0], _rot_MoI.size(), 1) = _rot_MoI;
    Eigen::Map<Eigen::VectorXd>(&_red_ratio_vect[0], _red_ratio.size(), 1) = _red_ratio;
    Eigen::Map<Eigen::VectorXd>(&_tau_req_vect[0], _tau_req.size(), 1) = _tau_req;

    iq_est_msg->msg().iq_est = _iq_est_vect;
    iq_est_msg->msg().q_p_ddot_est = _q_p_ddot_est_vect;
    iq_est_msg->msg().q_p_dot_meas = _q_p_dot_meas_vect;
    iq_est_msg->msg().tau_meas = _tau_meas_vect;
    iq_est_msg->msg().K_t = _K_t_vect;
    iq_est_msg->msg().K_d0 = _K_d0_vect;
    iq_est_msg->msg().K_d1 = _K_d1_vect;
    iq_est_msg->msg().rot_MoI = _rot_MoI_vect;
    iq_est_msg->msg().red_ratio = _red_ratio_vect;
    iq_est_msg->msg().tau_req = _tau_req_vect;

    iq_est_msg->msg().der_est_order = _der_est_order;

    iq_est_msg->msg().iq_jnt_names = _iq_jnt_names;

    _iq_est_pub->publishLoaned(std::move(iq_est_msg));

}

bool IqModelCalibRt::on_initialize()
{ 
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _jnt_names = _robot->getEnabledJointNames();;
    init_vars();

    init_nrt_ros_bridge();

    // using the order given by _jnt_names
//    _iq_getter = IqRosGetter(_verbose); // object to get the
    // quadrature current from XBot2 ROS topic (they need to be activated
    // manually)
    _iq_jnt_names = _jnt_names; // we will get (and estimate) the iq
//    _iq_getter.set_jnt_names(_iq_jnt_names);

    _iq_estimator = IqEstimator(_K_t,
                                _K_d0, _K_d1,
                                _rot_MoI,
                                _red_ratio); // object to compute the
    // iq estimate

    _num_diff = NumDiff(_n_jnts_robot, _plugin_dt, _der_est_order);

    return true;
    
}

void IqModelCalibRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    update_state(); // read current jnt positions and velocities

    start_completed(); // Move on to run()

}

void IqModelCalibRt::run()
{  

    update_state(); // update all necessary states

    _queue.run();

    _iq_estimator.set_current_state(_q_p_dot_meas, _q_p_ddot_est, _tau_meas);

    _iq_estimator.get_iq_estimate(_iq_est);

    pub_iq_est(); // publish estimates to topic

//    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

    if (_is_first_run)
    { // next control loops are aware that it is not the first control loop
        _is_first_run = !_is_first_run;
    }

    add_data2dump_logger();


}

void IqModelCalibRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void IqModelCalibRt::stopping()
{
    stop_completed();
}

void IqModelCalibRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void IqModelCalibRt::on_close()
{
    jinfo("Closing IqModelCalibRt");
}

XBOT2_REGISTER_PLUGIN(IqModelCalibRt, iq_model_calib_rt)

