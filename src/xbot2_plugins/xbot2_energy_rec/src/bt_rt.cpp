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

    _plugin_manager_name = getParamOrThrow<std::string>("~plugin_manager_name");

    _plugins_stat_topicname = getParamOrThrow<std::string>("~plugins_stat_topicname");

    _queue_size = getParamOrThrow<int>("~queue_size");

    _tree_logpath = getParamOrThrow<std::string>("~tree_logpath");

    _use_zmq_pub = getParamOrThrow<bool>("~use_zmq_pub");
    _use_bt_log =  getParamOrThrow<bool>("~use_bt_log");

    _stop_on_completion = getParamOrThrow<bool>("~stop_on_completion");

    _stop_ticking_root_if_completed = getParamOrThrow<bool>("~stop_ticking_root_if_completed");

    _verbose = getParamOrThrow<bool>("~verbose");

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

void BtRt::is_dummy(std::string dummy_string = "dummy")
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

    // ros publisher for root status
    _bt_root_status_pub = _ros->advertise<awesome_leg::BtRootStatus>(_bt_root_topicname + "/status", 1);

}

void BtRt::init_bt()
{

    _factory.registerNodeType<StartPlugins>("StartPlugins");
    _factory.registerNodeType<Set2Idle>("CoolDownByWaiting");
    _factory.registerNodeType<RestartJumpSequence>("RestartJumpSequence");
    _factory.registerNodeType<StopPlugins>("CloseAllPlugins");
    _factory.registerNodeType<Go2TakeoffConfig>("Go2TakeoffConfig");
    _factory.registerNodeType<PerformTakeoff>("PerformTakeoff");
    _factory.registerNodeType<Set2Idle>("WaitABit");

    _factory.registerNodeType<ArePluginsRunning>("ArePluginsRunning");
    _factory.registerNodeType<ArePluginsClosed>("ArePluginsClosed");
    _factory.registerNodeType<TemperatureOk>("TemperatureOk");
    _factory.registerNodeType<RecovEnergyReached>("RecovEnergyReached");
    _factory.registerNodeType<IsIdle>("IsIdle");

    _factory.registerNodeType<RestartJumpSequence>("RestartJumpSequence1");

//    _factory.registerNodeType<TakeoffReached>("TakeoffConfigReached");
//    _factory.registerNodeType<TakeoffPerformed>("TakeoffPerformed");

//    _factory.registerNodeType<Set2Idle>("WaitABit1");
//    _factory.registerNodeType<Set2Idle>("WaitABit2");
//    _factory.registerNodeType<Set2Idle>("WaitABit3");

    _factory.registerNodeType<PauseExpired>("PauseExpired1");
    _factory.registerNodeType<PauseExpired>("PauseExpired2");
    _factory.registerNodeType<PauseExpired>("PauseExpired3");


//    _factory.registerNodeType<StartPlugins>("StartPlugins");
//    _factory.registerNodeType<Set2Idle>("wait_for_actuators_cooling");

//    _factory.registerNodeType<ArePluginsRunning>("ArePluginsRunning");
//    _factory.registerNodeType<TemperatureOk>("temperature_ok");

    _tree = _factory.createTreeFromFile(_bt_description_path);

//    // This logger prints state changes on console
//    StdCoutLogger _logger_cout(_tree);

    // This logger saves state changes on file
    if (_use_bt_log)
    {
        _tree_log_fullpaths = _tree_logpath + "/" + _tree_logname + ".fbl";
        _logger_file = std::make_unique<FileLogger>(_tree, _tree_log_fullpaths.c_str());
    }
    
//    // This logger stores the execution time of each node
//    MinitraceLogger _logger_minitrace(_tree, "/tmp/bt_trace.json"); // causes segfault

    if(_use_zmq_pub)
    {
        // This logger publish status changes using ZeroMQ. Used by Groot
        _zmq_pub_ptr = std::make_unique<PublisherZMQ>(_tree);
    }

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                       "\nBT INFO: \n");

    printTreeRecursively(_tree.rootNode()); // display tree info

    jhigh().jprint("\n");

}

std::string BtRt::bt_status2string(NodeStatus status)
{

    if (status == NodeStatus::IDLE)
    {
        return "IDLE";
    }
    if (status == NodeStatus::RUNNING)
    {
        return "RUNNING";
    }
    if (status == NodeStatus::FAILURE)
    {
        return "FAILURE";
    }
    if (status == NodeStatus::SUCCESS)
    {
        return "SUCCESS";
    }
    else
    {
        return "INVALID";
    }

}

void BtRt::pub_bt_status()
{
    _bt_root_status_msg.status = bt_status2string(_bt_root_status);

    _bt_root_status_pub->publish(_bt_root_status_msg);
}

void BtRt::init_plugin_manager()
{

    _plugins_man_strt_req_pubs->publish(Command::Start);

    _plugins_man_strt_res_subs->run(); // processes plugins command service feedback

}

bool BtRt::on_initialize()
{
    std::string sim_flagname = "sim";
    is_sim(sim_flagname); // see if we are running a simulation

    std::string dummy_flagname = "dummy";
    is_dummy(dummy_flagname); // see if we are running in dummy mode

    read_config_from_yaml(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    init_vars();

    init_nrt_ros_bridge();

    create_ros_api();

    init_bt();

    // plugin manager asynchronous starting
    auto res_callback_cmd = [this](const bool& msg)
    {

        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                           "Received command response {} from plugin manager\n",
                           msg);

    };

    _plugins_man_strt_req_pubs = advertise<Runnable::Command>(_async_service_pattern + _plugin_manager_name + "/command/request");
    _plugins_man_strt_res_subs = subscribe<bool>(_async_service_pattern + _plugin_manager_name + "/command/response",
                                res_callback_cmd,
                                _queue_size);

    return true;

}

void BtRt::starting()
{

    init_dump_logger(); // needs to be here

    init_clocks(); // initialize clocks timers

    init_plugin_manager();

    // Move on to run()
    start_completed();

}

void BtRt::run()
{

    if(!_bt_finished)
    {// continue ticking root

        _bt_root_status = _tree.tickRoot();

    }

    _bt_root_status = _tree.tickRoot();

    if(_verbose)
    {
        jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                           "Root status: {}\n",
                           _bt_root_status);
    }

    add_data2dump_logger(); // add data to the logger

    pub_bt_status(); // publishes bt status to external ros topic

    update_clocks(); // last, update the clocks (loop + any additional one)

    if((_bt_root_status == NodeStatus::SUCCESS || _bt_root_status == NodeStatus::FAILURE) && _stop_ticking_root_if_completed)
    { // behaviour tree succeded --> exit

        _bt_finished = true;

        if(_stop_on_completion)
        {
            stop();

        }

    }

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
