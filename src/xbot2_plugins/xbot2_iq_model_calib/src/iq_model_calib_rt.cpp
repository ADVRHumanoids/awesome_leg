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

    _iq_est.reserve(_n_jnts_robot);
    _iq_jnt_names.reserve(_n_jnts_robot);

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
    _mat_name = getParamOrThrow<std::string>("~mat_name"); 
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix"); 
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _verbose = getParamOrThrow<bool>("~verbose");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");
    _K_t = getParamOrThrow<Eigen::VectorXd>("~K_t");
    _K_d0 = getParamOrThrow<Eigen::VectorXd>("~K_d0");
    _K_d1 = getParamOrThrow<Eigen::VectorXd>("~K_d1");
    _rot_MoI = getParamOrThrow<Eigen::VectorXd>("~rotor_axial_MoI");

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
    _num_diff.add_sample(_q_p_dot_meas, _plugin_dt);
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
    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);

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

    std::vector<float> iq_est_prealloc(_n_jnts_robot);
    std::vector<std::string> iq_jnt_names_prealloc(_n_jnts_robot);

    iq_status_prealloc.iq_est = iq_est_prealloc;
    iq_status_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    _iq_est_pub = _ros->advertise<awesome_leg::IqEstStatus>(
        "iq_est_node", 1, iq_status_prealloc);

}

void IqModelCalibRt::add_data2dump_logger()
{

    _dump_logger->add("q_p_meas", _q_p_meas);
    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("tau_meas", _tau_meas);

}

void IqModelCalibRt::add_data2bedumped()
{

    add_data2dump_logger();
    
}

void IqModelCalibRt::pub_iq_est()
{
    auto iq_est_msg = _iq_est_pub->loanMessage();

    iq_est_msg->msg().iq_est = _iq_est;
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

    init_vars();

    init_nrt_ros_bridge();

    _iq_getter = IqRosGetter(_verbose);

    _iq_estimator = IqEstimator(_K_t,
                                _K_d0, _K_d1,
                                _rot_MoI,
                                _red_ratio);

    _num_diff = NumDiff(_n_jnts_robot);

    return true;
    
}

void IqModelCalibRt::starting()
{

//    init_dump_logger(); // needs to be here

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

