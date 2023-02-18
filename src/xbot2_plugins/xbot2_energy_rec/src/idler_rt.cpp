#include "idler_rt.h"
#include "utils_defs.hpp"
#include <iostream>

#include <math.h>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

void IdlerRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void IdlerRt::reset_flags()
{

}

void IdlerRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void IdlerRt::get_params_from_config()
{

//    _queue_size = getParamOrThrow<int>("~queue_size");

    _verbose = getParamOrThrow<bool>("~verbose");

//    _mat_path = getParamOrThrow<std::string>("~mat_path");
//    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
//    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

//    _idle_status_topicname = getParamOrThrow<std::string>("~idle_status_topicname");

//    _idler_servicename = getParamOrThrow<std::string>("~idler_servicename");

}

void IdlerRt::is_sim(std::string sim_string = "sim")
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

void IdlerRt::is_dummy(std::string dummy_string = "dummy")
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

void IdlerRt::init_dump_logger()
{

    // // Initializing logger for debugging
    MatLogger2::Options opt;
    opt.default_buffer_size = _matlogger_buffer_size; // set default buffer size
    opt.enable_compression = true; // enable ZLIB compression
    if (_dump_mat_suffix == std::string(""))
    { // if empty, then dump to tmp with plugin name
        _dump_logger = MatLogger2::MakeLogger("/tmp/IdlerRt", opt); // date-time automatically appended
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
    _dump_logger->add("is_dummy", int(_is_dummy));
    _dump_logger->add("verbose", int(_verbose));

    _dump_logger->create("idle_status", 1, 1, _matlogger_buffer_size);
    _dump_logger->create("safety_stop", 1, 1, _matlogger_buffer_size);

}

void IdlerRt::add_data2dump_logger()
{
    _dump_logger->add("idle_status", int(_idle_state_msg.idle));
    _dump_logger->add("safety_stop", int(_safety_stop_msg.stop));

}


void IdlerRt::add_data2bedumped()
{

    add_data2dump_logger();

}

void IdlerRt::pub_state()
{

    _idle_state_pub->publish(_idle_state_msg);
    _safety_stop_pub->publish(_safety_stop_msg);

}

void IdlerRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    _idle_state_setter_srvr = _ros->advertiseService(_idler_servicename,
                                                    &IdlerRt::on_idle_cmd_received,
                                                    this,
                                                    &_queue);

    _stop_state_setter_srvr = _ros->advertiseService(_safety_stop_servicename,
                                                    &IdlerRt::on_safety_stop_cmd_received,
                                                    this,
                                                    &_queue);

    _idle_state_msg.idle = false;
    _idle_state_pub = _ros->advertise<awesome_leg::IdleState>(
        _idle_status_topicname, 1);

    _safety_stop_msg.stop = false;
    _safety_stop_pub = _ros->advertise<awesome_leg::SafetyStopState>(
        _safety_stop_status_topicname, 1);

}

bool IdlerRt::on_initialize()
{
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    get_params_from_config(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    init_nrt_ros_bridge();

    return true;

}

bool IdlerRt::on_idle_cmd_received(const awesome_leg::SetIdleStateRequest& req, awesome_leg::SetIdleStateResponse& res)
{
    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\nIdlerRt: received idle request with value: idle: {}.\n", req.idle);
    }

    _idle_state_msg.idle = req.idle;

    return res.success;
}

bool IdlerRt::on_safety_stop_cmd_received(const awesome_leg::SetSafetyStopRequest& req, awesome_leg::SetSafetyStopResponse& res)
{
    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                   "\nIdlerRt: received safety stop request with value: stop: {}.\n", req.stop);
    }

    _safety_stop_msg.stop = req.stop;

    return res.success;
}

void IdlerRt::starting()
{

    init_dump_logger(); // needs to be here

    reset_flags(); // reset flags, just in case

    init_clocks(); // initialize clocks timers

    start_completed(); // Move on to run()


}

void IdlerRt::run()
{

    _queue.run(); // process service callbacks

    // publish iq estimate info
    pub_state(); // publish to topic

    add_data2bedumped(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void IdlerRt::on_stop()
{
    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void IdlerRt::stopping()
{
    stop_completed();
}

void IdlerRt::on_abort()
{
    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void IdlerRt::on_close()
{
    jinfo("Closing IdlerRt");
}

XBOT2_REGISTER_PLUGIN(IdlerRt, idler_rt)
