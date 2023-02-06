#include "temp_monitor_rt.h"

#include <math.h>

void TempMonRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void TempMonRt::init_vars()
{
    _driver_temp_thresholds = Eigen::VectorXd::Zero(_n_jnts_robot);
    _meas_driver_temp = Eigen::VectorXd::Zero(_n_jnts_robot);

    for(int i = 0; i <_n_jnts_robot; i++)
    {
        _driver_temp_thresholds(i) = _driver_temp_threshold;

        _meas_driver_temp(i) = _fake_starting_temp;
    }
}

void TempMonRt::reset_flags()
{

}

void TempMonRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void TempMonRt::get_params_from_config()
{

    _temp_stat_topicname = getParamOrThrow<std::string>("~temp_stat_topicname");

    _queue_size = getParamOrThrow<int>("~queue_size");

    _driver_temp_threshold = getParamOrThrow<double>("~driver_temp_threshold");

    _verbose = getParamOrThrow<bool>("~verbose");

    _mat_path = getParamOrThrow<std::string>("~mat_path");
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _idle_status_topicname = getParamOrThrow<std::string>("~idle_status_topicname");

}

void TempMonRt::is_sim(std::string sim_string = "sim")
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

void TempMonRt::update_state()
{
    // "sensing" the robot
    _robot->sense();

    _robot->getTemperature(_meas_driver_temp); // getting driver temperature

}

void TempMonRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/TempMonRt", opt); // date-time automatically appended
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
    _dump_logger->add("driver_temp_threshold", _driver_temp_threshold);
    _dump_logger->add("verbose", int(_verbose));

    _dump_logger->create("is_drivers_temp_ok", 1, 1, _matlogger_buffer_size);
}

void TempMonRt::add_data2dump_logger()
{

}

void TempMonRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

}

void TempMonRt::add_data2bedumped()
{

    add_data2dump_logger();

}

void TempMonRt::check_driver_temp_limits()
{

    _is_drivers_temp_ok = (_meas_driver_temp.array() < _driver_temp_thresholds.array()).all();

    if(!_is_drivers_temp_ok)
    {
        if(_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::red),
                       "\n TempMonRt: driver temperature threshold of {} deg exceeded on one or more actuators.\n", _driver_temp_threshold);
        }
    }
}

void TempMonRt::pub_temp_status()
{

    _temp_ok_pub->publish(_is_drivers_temp_ok);

}

void TempMonRt::fake_temperature()
{

    if(!_is_idle)
    { // we increment the temperature linearly, if the robot is not in idle
        _meas_driver_temp = _meas_driver_temp.array() + _temp_rise_rate * _loop_time;
    }
    if(_is_idle)
    { // we are in idle and the actuators are cooling down
        _meas_driver_temp = _meas_driver_temp.array() + _temp_cooling_rate * _loop_time;
    }
}

bool TempMonRt::on_initialize()
{
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_jnts_robot = _robot->getJointNum();
    _jnt_names = _robot->getEnabledJointNames();;

    init_vars();

    init_nrt_ros_bridge();

    _temp_ok_pub = advertise<bool>(_temp_stat_topicname);

    // lambda to define callback
    auto on_idle_status_received = [this](const awesome_leg::SetPlugins2Idle& msg)
    {
        if(_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::red),
                       "\n TempMonRt: received idle status: {}.\n", msg.idle);
        }

        _idle_status_msg.idle= msg.idle;

    };

    _idle_status_sub = subscribe<awesome_leg::SetPlugins2Idle>(_idle_status_topicname,
                                                                on_idle_status_received,
                                                                _queue_size);
    return true;

}

void TempMonRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    update_state(); // read current jnt positions and velocities

    start_completed(); // Move on to run()

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::red),
                   "\n TempMonRt: current drivers temperature: \n", _meas_driver_temp);
    }

}

void TempMonRt::run()
{

    update_state(); // update all necessary states

    check_driver_temp_limits();

    _queue.run();

    if(_is_sim && _simulate_temp_if_sim)
    {
        fake_temperature();
    }

    // publish iq estimate info
    pub_temp_status(); // publish estimates to topic

    add_data2bedumped(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void TempMonRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void TempMonRt::stopping()
{
    stop_completed();
}

void TempMonRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void TempMonRt::on_close()
{
    jinfo("Closing TempMonRt");
}

XBOT2_REGISTER_PLUGIN(TempMonRt, temp_monitor_rt)
