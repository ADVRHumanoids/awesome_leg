#include "bus_power_rt.h"

#include <math.h>

void BusPowerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void BusPowerRt::init_vars()
{

    _q_p_meas = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_dot_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_dot_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _tau_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_meas = Eigen::VectorXd::Zero(_n_jnts_robot);
    _iq_meas_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _q_p_ddot_est = Eigen::VectorXd::Zero(_n_jnts_robot);
    _q_p_ddot_est_filt = Eigen::VectorXd::Zero(_n_jnts_robot);

    _iq_est = Eigen::VectorXd::Zero(_n_jnts_robot);
    _iq_jnt_names = std::vector<std::string>(_n_jnts_robot);

    _iq_friction_torque = Eigen::VectorXd::Zero(_n_jnts_robot);

    _tau_rot_est = Eigen::VectorXd::Zero(_n_jnts_robot);

    _dummy_eig_scalar = Eigen::VectorXd::Zero(1);

    // used to convert to ros messages-compatible types
    _iq_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_est_vect = std::vector<double>(_n_jnts_robot);
    _q_p_ddot_est_filt_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_meas_vect = std::vector<double>(_n_jnts_robot);
    _q_p_dot_meas_filt_vect = std::vector<double>(_n_jnts_robot);
    _tau_meas_vect = std::vector<double>(_n_jnts_robot);
    _K_t_vect = std::vector<double>(_n_jnts_robot);
    _K_d0_vect = std::vector<double>(_n_jnts_robot);
    _K_d1_vect = std::vector<double>(_n_jnts_robot);
    _rot_MoI_vect = std::vector<double>(_n_jnts_robot);
    _red_ratio_vect = std::vector<double>(_n_jnts_robot);
    _iq_friction_torque_vect = std::vector<double>(_n_jnts_robot);
    _tau_meas_filt_vect = std::vector<double>(_n_jnts_robot);

    _iq_meas_vect = std::vector<double>(_n_jnts_robot);
    _iq_meas_filt_vect = std::vector<double>(_n_jnts_robot);

    _tau_rot_est_vect = std::vector<double>(_n_jnts_robot);

    // regeneration stuff
    _er_k = Eigen::VectorXd::Zero(_n_jnts_robot);
    _pr_k = Eigen::VectorXd::Zero(_n_jnts_robot);
    _recov_energy = Eigen::VectorXd::Zero(_n_jnts_robot);
    _pk_joule = Eigen::VectorXd::Zero(_n_jnts_robot);
    _pk_mech = Eigen::VectorXd::Zero(_n_jnts_robot);
    _pk_indct = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ek_joule = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ek_mech = Eigen::VectorXd::Zero(_n_jnts_robot);
    _ek_indct = Eigen::VectorXd::Zero(_n_jnts_robot);

    _er_k_vect = std::vector<double>(_n_jnts_robot);
    _pr_k_vect = std::vector<double>(_n_jnts_robot);
    _recov_energy_vect = std::vector<double>(_n_jnts_robot);
    _pk_joule_vect = std::vector<double>(_n_jnts_robot);
    _pk_mech_vect = std::vector<double>(_n_jnts_robot);
    _pk_indct_vect = std::vector<double>(_n_jnts_robot);
    _ek_joule_vect = std::vector<double>(_n_jnts_robot);
    _ek_mech_vect = std::vector<double>(_n_jnts_robot);
    _ek_indct_vect = std::vector<double>(_n_jnts_robot);


}

void BusPowerRt::reset_flags()
{

}

void BusPowerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void BusPowerRt::get_params_from_config()
{
    // Reading some parameters from XBot2 config. YAML file

    _mat_path = getParamOrThrow<std::string>("~mat_path");
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _red_ratio = getParamOrThrow<Eigen::VectorXd>("~red_ratio");
    _K_t = getParamOrThrow<Eigen::VectorXd>("~K_t");
    _K_d0 = getParamOrThrow<Eigen::VectorXd>("~K_d0");
    _K_d1 = getParamOrThrow<Eigen::VectorXd>("~K_d1");
    _rot_MoI = getParamOrThrow<Eigen::VectorXd>("~rotor_axial_MoI");

    _der_est_order = getParamOrThrow<int>("~der_est_order");
    _mov_avrg_cutoff_freq_iq = getParamOrThrow<double>("~mov_avrg_cutoff_freq_iq");
    _mov_avrg_cutoff_freq_tau = getParamOrThrow<double>("~mov_avrg_cutoff_freq_tau");
    _mov_avrg_cutoff_freq_iq_meas = getParamOrThrow<double>("~mov_avrg_cutoff_freq_iq_meas");
    _mov_avrg_cutoff_freq_q_dot = getParamOrThrow<double>("~mov_avrg_cutoff_freq_q_dot");

    _alpha = getParamOrThrow<int>("~alpha");

    _q_dot_3sigma = getParamOrThrow<double>("~q_dot_3sigma");

    _R = getParamOrThrow<Eigen::VectorXd>("~R");
    _L_m = getParamOrThrow<Eigen::VectorXd>("~L_m");
    _L_leak = getParamOrThrow<Eigen::VectorXd>("~L_leak");

    _use_iq_meas = getParamOrThrow<bool>("~use_iq_meas");

    _dump_iq_data = getParamOrThrow<bool>("~dump_iq_data");

    _topic_ns = getParamOrThrow<std::string>("~topic_ns");

    _set_monitor_state_servname = getParamOrThrow<std::string>("~set_monitor_state_servname");

    _verbose = getParamOrThrow<bool>("~verbose");

    _bus_p_leak = getParamOrThrow<double>("~bus_p_leak");

}

void BusPowerRt::is_sim(std::string sim_string = "sim")
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

void BusPowerRt::is_dummy(std::string dummy_string = "dummy")
{
    XBot::Context ctx;
    auto& pm = ctx.paramManager();
    _hw_type = pm.getParamOrThrow<std::string>("/xbot_internal/hal/hw_type");

    size_t dummy_found = _hw_type.find(dummy_string);

    if (dummy_found != std::string::npos) { // we are running the plugin in dummy mode

        _is_dummy = true;
    }
    else // we are running on the real robot
    {
        _is_dummy = false;
    }

}

void BusPowerRt::update_state()
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

    // Removing noise from q_p_dot (mostly used for introducing a uniform lag
    // w.r.t. other data)
    _mov_avrg_filter_q_dot.add_sample(_q_p_dot_meas);
    _mov_avrg_filter_q_dot.get(_q_p_dot_meas_filt);

    // Removing noise from q_p_ddot
    _mov_avrg_filter_iq.add_sample(_q_p_ddot_est);
    _mov_avrg_filter_iq.get(_q_p_ddot_est_filt);

    // Removing noise from tau_meas
    _mov_avrg_filter_tau.add_sample(_tau_meas);
    _mov_avrg_filter_tau.get(_tau_meas_filt);

    //getting iq measure
    if(_use_iq_meas)
    {
        _iq_getter->get_last_iq_out(_iq_meas);
        _iq_getter->get_last_iq_out_filt(_iq_meas_filt);
    }

}

void BusPowerRt::update_sensed_power()
{
    #if defined(ON_REAL_ROBOT)

    if(_power_sensor_found)
    {
        _vbatt = _pow_sensor->get_battery_voltage();
        _ibatt = - _pow_sensor->get_load_current(); // current is measured positive if flowing towards the robot
        // hence, if we want the current goint into the battery, we have to switch sign
    }

    #endif

    _p_batt = _vbatt * _ibatt;

    _dummy_eig_scalar(0) = _p_batt;
    _meas_pow_int.add_sample(_dummy_eig_scalar);
    _meas_pow_int.get(_dummy_eig_scalar);
    _e_batt = _dummy_eig_scalar(0);

    if (_enable_meas_rec_energy_monitoring && _p_batt > 0)
    {// only integrate if monitoring reg. power and if power is flowing towards the battery
        _dummy_eig_scalar(0) = _p_batt;
        _reg_meas_pow_int.add_sample(_dummy_eig_scalar);
        _reg_meas_pow_int.get(_dummy_eig_scalar);
        _reg_energy = _dummy_eig_scalar(0);
    }
}

void BusPowerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/BusPowerRt", opt); // date-time automatically appended
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
    _dump_logger->create("q_p_dot_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("q_p_ddot_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->add("der_est_order", _der_est_order);

    _dump_logger->create("q_p_ddot_est_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("tau_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("iq_meas", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("iq_meas_filt", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("iq_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("iq_friction_torque", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("K_t", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d0", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("K_d1", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("rot_MoI", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("red_ratio", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->create("tau_total_rot", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("tau_rot_est", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("alpha_f0", _n_jnts_robot, 1, _matlogger_buffer_size);
    _dump_logger->create("alpha_f1", _n_jnts_robot, 1, _matlogger_buffer_size);

    _dump_logger->add("mov_avrpub_meas_reg_powg_window_size_iq", _mov_avrg_window_size_iq);
    _dump_logger->add("mov_avrg_cutoff_freq_iq", _mov_avrg_cutoff_freq_iq);

    _dump_logger->add("mov_avrg_window_size_tau", _mov_avrg_window_size_iq);
    _dump_logger->add("mov_avrg_cutoff_freq_tau", _mov_avrg_cutoff_freq_iq);

    // power sensor
    _dump_logger->create("vbatt", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("ibatt", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("e_batt", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("p_batt", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("reg_power", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("reg_energy", 1, 1, _matlogger_buffer_size);

}

void BusPowerRt::add_data2dump_logger()
{

    _dump_logger->add("q_p_meas", _q_p_meas);

    _dump_logger->add("q_p_dot_meas", _q_p_dot_meas);
    _dump_logger->add("q_p_dot_meas_filt", _q_p_dot_meas_filt);

    _dump_logger->add("tau_meas", _tau_meas);
    _dump_logger->add("tau_meas_filt", _tau_meas_filt);

    _dump_logger->add("q_p_ddot_est", _q_p_ddot_est);
    _dump_logger->add("q_p_ddot_est_filt", _q_p_ddot_est_filt);

    _dump_logger->add("iq_meas", _iq_meas);
    _dump_logger->add("iq_meas_filt", _iq_meas_filt);

    _dump_logger->add("iq_est", _iq_est);
    _dump_logger->add("iq_friction_torque", _iq_friction_torque);

    _dump_logger->add("K_t", _K_t);
    _dump_logger->add("K_d0", _K_d0);
    _dump_logger->add("K_d1", _K_d1);
    _dump_logger->add("rot_MoI", _rot_MoI);
    _dump_logger->add("red_ratio", _red_ratio);

    _dump_logger->add("tau_rot_est", _tau_rot_est);

    // power sensor
    _dump_logger->add("vbatt", _vbatt);
    _dump_logger->add("ibatt", _ibatt);
    _dump_logger->add("e_batt", _e_batt);
    _dump_logger->add("p_batt", _p_batt);
    _dump_logger->add("reg_energy", _reg_energy);

}

void BusPowerRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    /* Subscribers */
    if(_use_iq_meas)
    {
        _aux_signals_sub = _ros->subscribe("/xbotcore/aux",
                                    &Xbot2Utils::IqRosGetter::on_aux_signal_received,
                                    _iq_getter.get(),
                                    1,  // queue size
                                    &_queue);

        _js_signals_sub = _ros->subscribe("/xbotcore/joint_states",
                                    &Xbot2Utils::IqRosGetter::on_js_signal_received,
                                    _iq_getter.get(),
                                    1,  // queue size
                                    &_queue);
    }
    /* Publishers */

    std::vector<std::string> iq_jnt_names_prealloc(_n_jnts_robot);

    // iq model status publisher
    awesome_leg::IqEstStatus iq_status_prealloc;

    std::vector<double> iq_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_ddot_est_prealloc(_n_jnts_robot);
    std::vector<double> q_p_ddot_est_filt_prealloc(_n_jnts_robot);
    std::vector<double> q_p_dot_meas_prealloc(_n_jnts_robot);
    std::vector<double> tau_meas_prealloc(_n_jnts_robot);
    std::vector<double> Kt_prealloc(_n_jnts_robot);
    std::vector<double> Kd0_prealloc(_n_jnts_robot);
    std::vector<double> Kd1_prealloc(_n_jnts_robot);
    std::vector<double> rot_MoI_prealloc(_n_jnts_robot);
    std::vector<double> red_ratio_prealloc(_n_jnts_robot);
    std::vector<double> iq_friction_torque_prealloc(_n_jnts_robot);

    iq_status_prealloc.iq_est = iq_est_prealloc;
    iq_status_prealloc.q_p_ddot_est = q_p_ddot_est_prealloc;
    iq_status_prealloc.q_p_ddot_est_filt = q_p_ddot_est_prealloc;
    iq_status_prealloc.q_p_dot_meas = q_p_dot_meas_prealloc;
    iq_status_prealloc.tau_meas = tau_meas_prealloc;
    iq_status_prealloc.K_t = Kt_prealloc;
    iq_status_prealloc.K_d0 = Kd0_prealloc;
    iq_status_prealloc.K_d1 = Kd1_prealloc;
    iq_status_prealloc.rot_MoI = rot_MoI_prealloc;
    iq_status_prealloc.red_ratio = red_ratio_prealloc;
    iq_status_prealloc.tau_friction = iq_friction_torque_prealloc;
    iq_status_prealloc.der_est_order = _der_est_order;

    iq_status_prealloc.iq_jnt_names = iq_jnt_names_prealloc;
    
    std::string iq_est_topic = "iq_est_node_pow_" + _topic_ns;

    _iq_est_pub = _ros->advertise<awesome_leg::IqEstStatus>(
        iq_est_topic.c_str(), 1, iq_status_prealloc);

    // regenerative pow. status publisher

    awesome_leg::EstRegPowStatus reg_pow_prealloc;

    double er_prealloc = 0.0, pr_prealloc = 0.0;
    std::vector<double> er_k_prealloc(_n_jnts_robot);
    std::vector<double> pr_k_prealloc(_n_jnts_robot);
    std::vector<double> pk_joule_prealloc(_n_jnts_robot);
    std::vector<double> pk_mech_prealloc(_n_jnts_robot);
    std::vector<double> pk_indct_prealloc(_n_jnts_robot);
    std::vector<double> ek_joule_prealloc(_n_jnts_robot);
    std::vector<double> ek_mech_prealloc(_n_jnts_robot);
    std::vector<double> ek_indct_prealloc(_n_jnts_robot);

    reg_pow_prealloc.er = er_prealloc;
    reg_pow_prealloc.pr = pr_prealloc;
    reg_pow_prealloc.er_k = er_k_prealloc;
    reg_pow_prealloc.pr_k = pr_k_prealloc;
    reg_pow_prealloc.pk_joule = pk_joule_prealloc;
    reg_pow_prealloc.pk_mech = pk_mech_prealloc;
    reg_pow_prealloc.pk_indct = pk_indct_prealloc;
    reg_pow_prealloc.ek_joule = ek_joule_prealloc;
    reg_pow_prealloc.ek_mech = ek_mech_prealloc;
    reg_pow_prealloc.ek_indct = ek_indct_prealloc;

    reg_pow_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    std::string est_reg_pow_topic = "est_reg_pow_node_" + _topic_ns;

    _est_reg_pow_pub = _ros->advertise<awesome_leg::EstRegPowStatus>(est_reg_pow_topic.c_str(), 1, reg_pow_prealloc);

    // regenerative pow. sensor status publisher
    awesome_leg::MeasRegPowStatus meas_reg_pow_prealloc;

    double vbatt_prealloc(_n_jnts_robot);
    double iload_prealloc(_n_jnts_robot);
    double e_batt_prealloc(_n_jnts_robot);
    double p_batt_prealloc(_n_jnts_robot);
    double reg_power_prealloc(_n_jnts_robot);
    double reg_energy_prealloc(_n_jnts_robot);

    meas_reg_pow_prealloc.vbatt = vbatt_prealloc;
    meas_reg_pow_prealloc.ibatt = iload_prealloc;
    meas_reg_pow_prealloc.e_batt = e_batt_prealloc;
    meas_reg_pow_prealloc.p_batt = p_batt_prealloc;
    meas_reg_pow_prealloc.reg_energy = reg_energy_prealloc;

    std::string meas_reg_pow_topic = "meas_reg_pow_node";

    _meas_reg_pow_pub = _ros->advertise<awesome_leg::MeasRegPowStatus>(meas_reg_pow_topic.c_str(), 1, meas_reg_pow_prealloc);

    // iq measurement publisher

    awesome_leg::IqMeasStatus iq_meas_prealloc;
    iq_meas_prealloc.iq_jnt_names = iq_jnt_names_prealloc;

    std::string iq_meas_topic = "iq_meas_node_" + _topic_ns;
    
    _iq_meas_pub = _ros->advertise<awesome_leg::IqMeasStatus>(iq_meas_topic.c_str(), 1, iq_meas_prealloc);

    // iq monitoring service

    _set_monitoring_state_srvr = _ros->advertiseService(_set_monitor_state_servname,
                                                &BusPowerRt::on_monitor_state_signal,
                                                this,
                                                &_queue);


}

bool BusPowerRt::on_monitor_state_signal(const awesome_leg::SetRegEnergyMonitoringStatusRequest& req, awesome_leg::SetRegEnergyMonitoringStatusResponse& res)
{

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                      "\nBusPowerRt: received regenerative energy monitor trigger signal: {}\n", req.monitor_energy);
    }

    bool state_changed = _monitor_recov_energy != req.monitor_energy;

    bool result = state_changed? true : false;

    if(state_changed)
    {
         _monitor_recov_energy = req.monitor_energy;

        if(_monitor_recov_energy)
        {
            _pow_estimator->enable_rec_energy_monitoring(); // we enable energy recovery monitoring

            _enable_meas_rec_energy_monitoring = true; // we reset sensor readings

        }
        if(!_monitor_recov_energy)
        {
            _pow_estimator->disable_rec_energy_monitoring(); // we disable energy recovery monitoring

            _enable_meas_rec_energy_monitoring = false; // we reset sensor readings

        }

    }

    if(req.reset_energy)
    {
        _pow_estimator->reset_rec_energy(); // we reset the recovered energy level

        _reg_meas_pow_int.reset(); // we reset sensor readings
        _reg_energy = 0.0;

    }

    res.success = result;

    return result;

}

void BusPowerRt::add_data2bedumped()
{

    add_data2dump_logger();

    _pow_estimator->add2log();

    _iq_estimator->add2log();

}

void BusPowerRt::pub_iq_est()
{
    auto iq_est_msg = _iq_est_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published
    Eigen::Map<Eigen::VectorXd>(&_iq_est_vect[0], _iq_est.size(), 1) = _iq_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ddot_est_vect[0], _q_p_ddot_est.size(), 1) = _q_p_ddot_est;
    Eigen::Map<Eigen::VectorXd>(&_q_p_ddot_est_filt_vect[0], _q_p_ddot_est_filt.size(), 1) = _q_p_ddot_est_filt;
    Eigen::Map<Eigen::VectorXd>(&_q_p_dot_meas_vect[0], _q_p_dot_meas.size(), 1) = _q_p_dot_meas_filt;
    Eigen::Map<Eigen::VectorXd>(&_tau_meas_vect[0], _tau_meas_filt.size(), 1) = _tau_meas_filt;
    Eigen::Map<Eigen::VectorXd>(&_K_t_vect[0], _K_t.size(), 1) = _K_t;
    Eigen::Map<Eigen::VectorXd>(&_K_d0_vect[0], _K_d0.size(), 1) = _K_d0;
    Eigen::Map<Eigen::VectorXd>(&_K_d1_vect[0], _K_d1.size(), 1) = _K_d1;
    Eigen::Map<Eigen::VectorXd>(&_rot_MoI_vect[0], _rot_MoI.size(), 1) = _rot_MoI;
    Eigen::Map<Eigen::VectorXd>(&_red_ratio_vect[0], _red_ratio.size(), 1) = _red_ratio;
    Eigen::Map<Eigen::VectorXd>(&_iq_friction_torque_vect[0], _iq_friction_torque.size(), 1) = _iq_friction_torque;

    // filling message
    iq_est_msg->msg().iq_est = _iq_est_vect;
    iq_est_msg->msg().q_p_ddot_est = _q_p_ddot_est_vect;
    iq_est_msg->msg().q_p_ddot_est_filt = _q_p_ddot_est_filt_vect;
    iq_est_msg->msg().q_p_dot_meas = _q_p_dot_meas_vect;
    iq_est_msg->msg().tau_meas = _tau_meas_vect;
    iq_est_msg->msg().K_t = _K_t_vect;
    iq_est_msg->msg().K_d0 = _K_d0_vect;
    iq_est_msg->msg().K_d1 = _K_d1_vect;
    iq_est_msg->msg().rot_MoI = _rot_MoI_vect;
    iq_est_msg->msg().red_ratio = _red_ratio_vect;
    iq_est_msg->msg().tau_friction = _iq_friction_torque_vect;

    iq_est_msg->msg().der_est_order = _der_est_order;

    iq_est_msg->msg().iq_jnt_names = _iq_jnt_names;

    _iq_est_pub->publishLoaned(std::move(iq_est_msg));

}

void BusPowerRt::pub_est_reg_pow()
{

    auto reg_pow_msg = _est_reg_pow_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        _er_k_vect[i] = _er_k(i);
        _pr_k_vect[i] = _pr_k(i);
        _recov_energy_vect[i] = _recov_energy(i);
        _pk_joule_vect[i] = _pk_joule(i);
        _pk_mech_vect[i] = _pk_mech(i);
        _pk_indct_vect[i] = _pk_indct(i);
        _ek_joule_vect[i] = _ek_joule(i);
        _ek_mech_vect[i] = _ek_mech(i);
        _ek_indct_vect[i] = _ek_indct(i);
    }

    // filling message
    reg_pow_msg->msg().er = _er;
    reg_pow_msg->msg().pr = _pr;

    reg_pow_msg->msg().recov_energy_tot = _recov_energy_tot;

    reg_pow_msg->msg().er_k = _er_k_vect;
    reg_pow_msg->msg().pr_k = _pr_k_vect;
    reg_pow_msg->msg().recov_energy = _recov_energy_vect;

    reg_pow_msg->msg().pk_joule = _pk_joule_vect;
    reg_pow_msg->msg().pk_mech = _pk_mech_vect;
    reg_pow_msg->msg().pk_indct = _pk_indct_vect;
    reg_pow_msg->msg().ek_joule = _ek_joule_vect;
    reg_pow_msg->msg().ek_mech = _ek_joule_vect;
    reg_pow_msg->msg().ek_indct = _ek_joule_vect;

    reg_pow_msg->msg().iq_jnt_names = _iq_jnt_names;

    reg_pow_msg->msg().monitor_recov_energy = _monitor_recov_energy;

    _est_reg_pow_pub->publishLoaned(std::move(reg_pow_msg));

}

void BusPowerRt::pub_meas_reg_pow()
{
    auto reg_pow_msg = _meas_reg_pow_pub->loanMessage();

    reg_pow_msg->msg().vbatt = _vbatt;
    reg_pow_msg->msg().ibatt = _ibatt;
    reg_pow_msg->msg().e_batt = _e_batt;
    reg_pow_msg->msg().p_batt = _p_batt;
    reg_pow_msg->msg().reg_energy = _reg_energy;

    _meas_reg_pow_pub->publishLoaned(std::move(reg_pow_msg));

}

void BusPowerRt::pub_iq_meas()
{
    auto iq_meas_msg = _iq_meas_pub->loanMessage();

    // mapping EigenVectorXd data to std::vector, so that they can be published

    for(int i = 0; i < _n_jnts_robot; i++)
    {
        _iq_meas_vect[i] = _iq_meas(i);
        _iq_meas_filt_vect[i] = _iq_meas_filt(i);

    }

    // filling message
    iq_meas_msg->msg().iq_meas = _iq_meas_vect;
    iq_meas_msg->msg().iq_meas_filt = _iq_meas_filt_vect;
    iq_meas_msg->msg().iq_jnt_names = _iq_jnt_names;

    _iq_meas_pub->publishLoaned(std::move(iq_meas_msg));

}

void BusPowerRt::run_iq_estimation()
{

    // we update the state of the iq estimator with the most recent ones (filtered)
    _iq_estimator->set_current_state(_q_p_dot_meas_filt, _q_p_ddot_est_filt, _tau_meas_filt);

    // we compute and get the iq estimate using the (possibly) updated K_d0 and K_d1 gains
    _iq_estimator->update(_K_d0, _K_d1);

    // we also get other useful quantities from the estimator
    _iq_estimator->get_tau_friction(_iq_friction_torque);

    _iq_estimator->get_iq_estimate(_iq_est);

}

void BusPowerRt::run_reg_pow_estimation()
{
     _pow_estimator->update(); // update power and energy estimates using current state
     // and current estimates from iq estimator (or iq measurements if _use_iq_meas == true)

     _pow_estimator->get(_er_k, _pr_k);

     _pow_estimator->get_e(_er);
     _pow_estimator->get_p(_pr);

     _pow_estimator->get_p_terms(_pk_joule, _pk_mech, _pk_indct);
     _pow_estimator->get_e_terms(_ek_joule, _ek_mech, _ek_indct);

     _pow_estimator->get_current_e_recov(_recov_energy);
     _pow_estimator->get_current_e_recov(_recov_energy_tot);

}

bool BusPowerRt::on_initialize()
{
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _jnt_names = _robot->getEnabledJointNames();;

    init_vars();

    // using the order given by _jnt_names
    _iq_getter.reset(new IqRosGetter(_plugin_dt, 
                                    false,
                                    _mov_avrg_cutoff_freq_iq_meas));

    // quadrature current from XBot2 ROS topic (they need to be activated
    // manually)
    _iq_jnt_names = _jnt_names; // we will get (and estimate) the iq
    _iq_getter->set_jnt_names(_iq_jnt_names);

    // iq estimation model
    _iq_estimator.reset(new IqEstimator(_K_t,
                                        _K_d0, _K_d1,
                                        _rot_MoI,
                                        _red_ratio,
                                        _alpha,
                                        _q_dot_3sigma,
                                        _dump_iq_data));

    // bus power estimator
    _pow_estimator.reset(new RegEnergy(_iq_getter,
                                     _iq_estimator,
                                     _R,
                                     _L_leak, _L_m,
                                     _bus_p_leak,
                                     _plugin_dt,
                                     _use_iq_meas,
                                     _dump_iq_data));
    _pow_estimator->enable_rec_energy_monitoring(); // we enable energy recovery monitoring
    // by default

    // numerical differentiation
    _num_diff = NumDiff(_n_jnts_robot, _plugin_dt, _der_est_order);

    //* filtering --> we should filter everything, also the q_dot (which is not so noisy)
    // so that we get the same delay for each data necessary for the iq model calibration
    // (this means we should filter with the same cutoff frequency or window length)*//

    // filter for iq(estimate)
    _mov_avrg_filter_iq = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_iq.get_window_size(_mov_avrg_window_size_iq); // get computed window size

    //filter for tau_meas
    _mov_avrg_filter_tau = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size_tau); // get computed window size

    //filter for q_dot
    _mov_avrg_filter_q_dot = MovAvrgFilt(_n_jnts_robot, _plugin_dt, _mov_avrg_cutoff_freq_iq);
    _mov_avrg_filter_tau.get_window_size(_mov_avrg_window_size_q_dot); // get computed window size

    init_nrt_ros_bridge();

    #if defined(ON_REAL_ROBOT)

    // get power sensor handle
    _pow_sensor = _robot->getDevices<Hal::PowerBoardEc>().get_device("power_sensor");
    _power_sensor_found = _pow_sensor != nullptr ? true : false;

    #endif

    _meas_pow_int = NumIntRt(1, _plugin_dt);
    _reg_meas_pow_int = NumIntRt(1, _plugin_dt);

    return true;

}

void BusPowerRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    _monitor_recov_energy = false;
    _pow_estimator->disable_rec_energy_monitoring(); // the monitoring of
    // the recovered energy is in idle by defaults

    start_completed(); // Move on to run()

}

void BusPowerRt::run()
{

    update_state(); // update all necessary states

    _queue.run();

    // computing and updating iq estimates
    run_iq_estimation();

    // publish iq estimate info
    pub_iq_est(); // publish estimates to topic

    // computing and updating reg. power and energy estimates
    run_reg_pow_estimation();

    // computing measured power @power supply and energy
    update_sensed_power();

    // publish reg. pow. info
    pub_est_reg_pow();
    pub_meas_reg_pow();

    // publishes iq measurements to rostopic
    pub_iq_meas();

    add_data2bedumped(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void BusPowerRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();

    _pow_estimator->reset_rec_energy(); // we reset the recovered energy level

}

void BusPowerRt::stopping()
{
    stop_completed();
}

void BusPowerRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void BusPowerRt::on_close()
{
    jinfo("Closing BusPowerRt");
}

XBOT2_REGISTER_PLUGIN(BusPowerRt, bus_power_rt)
