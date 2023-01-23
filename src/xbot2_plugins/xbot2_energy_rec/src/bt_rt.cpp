#include "bt_rt.h"

void BtRt::init_clocks()
{
    _loop_time = 0.0; // reset loop time clock
}

void BtRt::init_vars()
{


}

void BtRt::update_clocks()
{
    // Update timer(s)
    _loop_time += _plugin_dt;

    // Reset timers, if necessary
    if (_loop_time >= _loop_timer_reset_time)
    {
        _loop_time = _loop_time - _loop_timer_reset_time;
    }

}

void BtRt::read_config_from_yaml()
{
    _mat_path = getParamOrThrow<std::string>("~mat_path");
    _dump_mat_suffix = getParamOrThrow<std::string>("~dump_mat_suffix");
    _matlogger_buffer_size = getParamOrThrow<double>("~matlogger_buffer_size");

    _bt_description_path = getParamOrThrow<std::string>("~bt_description_path");

}

void BtRt::is_sim(std::string sim_string = "sim")
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

void BtRt::create_ros_api()
{
    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "bt_rt");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "bt_rt");
}

void BtRt::init_dump_logger()
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

}

void BtRt::add_data2dump_logger()
{

}

void BtRt::init_nrt_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

}

void BtRt::init_bt()
{

    auto tree = _factory.createTreeFromFile(_bt_description_path);

}

bool BtRt::on_initialize()
{
    std::string sim_flagname = "sim";

    is_sim(sim_flagname); // see if we are running a simulation

    read_config_from_yaml(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    init_vars();

    init_nrt_ros_bridge();

    create_ros_api();

    return true;

}

void BtRt::starting()
{

    init_dump_logger(); // needs to be here

    init_clocks(); // initialize clocks timers

    // Move on to run()
    start_completed();

}

void BtRt::run()
{

    _queue.run();

    add_data2dump_logger(); // add data to the logger

    update_clocks(); // last, update the clocks (loop + any additional one)

}

void BtRt::on_stop()
{

    init_clocks();

    // Destroy logger and dump .mat file (will be recreated upon plugin restart)
    _dump_logger.reset();
}

void BtRt::stopping()
{
    stop_completed();
}

void BtRt::on_abort()
{

    // Destroy logger and dump .mat file
    _dump_logger.reset();
}

void BtRt::on_close()
{
    jinfo("Closing BtRt");
}

XBOT2_REGISTER_PLUGIN(BtRt, bt_rt)