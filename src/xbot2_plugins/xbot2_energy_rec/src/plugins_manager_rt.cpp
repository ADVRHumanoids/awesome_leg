#include "plugins_manager_rt.h"

void PluginsManagerRt::read_config_from_yaml()
{
    _plugin_list = getParamOrThrow<std::vector<std::string>>("~plugin_list");

//    _queue_size = getParamOrThrow<int>("~queue_size");

//    _async_service_pattern = getParamOrThrow<std::string>("~async_service_pattern");

//    _plugins_stat_topicname = getParamOrThrow<std::string>("~plugins_stat_topicname");

    _verbose = getParamOrThrow<bool>("~verbose");

}

void PluginsManagerRt::init_ros_bridge()
{
    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    _plugins_stat_pub = _ros->advertise<awesome_leg::PluginsManStatus>(
        _plugins_stat_topicname, 1);

    _start_plugins_srvr = _ros->advertiseService(_start_plugins_servname,
                                                &PluginsManagerRt::on_start_signal,
                                                this,
                                                &_queue);
    _stop_plugins_srvr = _ros->advertiseService(_stop_plugins_servname,
                                                &PluginsManagerRt::on_stop_signal,
                                                this,
                                                &_queue);

}
bool PluginsManagerRt::on_initialize()
{
    std::string sim_flagname = "sim";

    read_config_from_yaml(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_plugins = _plugin_list.size();

    _strt_req_pubs = std::vector<PublisherPtr< XBot::RpcWrapper<XBot::Runnable::Command>>>(_n_plugins);
    _strt_res_subs = std::vector<SubscriberPtr<XBot::RpcWrapper<bool>>>(_n_plugins);

    _status_req_pubs = std::vector<PublisherPtr<XBot::RpcWrapper<service::Empty>>>(_n_plugins);
    _status_res_subs = std::vector<SubscriberPtr<XBot::RpcWrapper<std::string>>>(_n_plugins);

    _plugins_status = std::vector<std::string>(_n_plugins);

    for(int i = 0; i < _n_plugins; i++)
    {

        // plugin starting/stopping
        auto res_callback_cmd = [i, this](const XBot::RpcWrapper<bool>& msg)
        {

            if(_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                                   "Received command response {} from plugin: {} \n",
                                   msg.obj, _plugin_list[i]);
            }


        };

        _strt_req_pubs[i] = advertise< XBot::RpcWrapper<XBot::Runnable::Command>>(_async_service_pattern + _plugin_list[i] + "/command/request");
        _strt_res_subs[i] = subscribe<XBot::RpcWrapper<bool>>(_async_service_pattern + _plugin_list[i] + "/command/response",
                                    res_callback_cmd,
                                    _queue_size);

        // plugin state getter
        auto res_callback_state = [i, this](const XBot::RpcWrapper<std::string>& msg)
        {

            _plugins_status[i] = msg.obj;

            if(_verbose)
            {
                jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                                  "Received status response {} from plugin: {} \n",
                                  msg.obj, _plugin_list[i]);
            }
        };

        _status_req_pubs[i] = advertise<XBot::RpcWrapper<service::Empty>>(_async_service_pattern + _plugin_list[i] + "/state/request");
        _status_res_subs[i] = subscribe<XBot::RpcWrapper<std::string>>(_async_service_pattern + _plugin_list[i] + "/state/response",
                                    res_callback_state,
                                    _queue_size_status);

        // inizializing knwon plugin state to false
        _plugins_status[i] = "";
    }

    init_ros_bridge();

    _start_command.obj = XBot::Runnable::Command::Start;
    _stop_command.obj = XBot::Runnable::Command::Stop;

    return true;

}

bool PluginsManagerRt::on_start_signal(const SimpleTriggerRequest& req, SimpleTriggerResponse& res)
{

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                      "\nPluginsManagerRt: received start signal!! \n");

    if(!_is_first_trigger)
    {
        res.success = _start_plugins? false: true; // failure if trigger variables already set
    }
    else
    {
        _is_first_trigger = false;
        res.success = true;
    }

    _start_plugins = true; // flag to start all plugins

    _triggered = true;

    return res.success;
}

bool PluginsManagerRt::on_stop_signal(const SimpleTriggerRequest& req, SimpleTriggerResponse& res)
{

    jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                      "\n PluginsManagerRt: received stop signal!! \n");

    if(!_is_first_trigger)
    {
        res.success = _start_plugins? true: false; // failure if trigger variables already set
    }
    else
    {
        _is_first_trigger = false;
        res.success = true;
    }

    _start_plugins = false; // flag to stop all plugins

    _triggered = true;

    return res.success;
}

void PluginsManagerRt::starting()
{

    // Move on to run()
    start_completed();

}

void PluginsManagerRt::run()
{

    _running_plugins_counter = 0;
    _stopped_plugins_counter = 0;

    for(int i = 0; i < _n_plugins; i++)
    { // we first read the status of the plugins (saved into _plugins_status)

        _status_req_pubs[i]->publish(_empty_msg);
        _status_res_subs[i]->run(); // processes plugins status callbacks

    }

    if(_start_plugins)
    { // in this case the plugin, when triggered, starts(if not already running) all the plugins in the list

        for(int i = 0; i < _n_plugins; i++)
        {

            if((_plugins_status[i] == "Initialized" || _plugins_status[i] == "Stopped"))
            {// initialized or stopped externally, but not running -> we start the plugin

                if(_triggered)
                {
                    if(_verbose)
                    {
                        jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                                           "Trying to start plugin {}\n",
                                           _plugin_list[i]);

                    }

                    _strt_req_pubs[i]->publish(_start_command);

                    _strt_res_subs[i]->run(); // processes plugins command service feedback
                }

            }
            else
            { // we enter here if State::Running or State::Aborted

                if(_plugins_status[i] == "Running")
                {
                    _running_plugins_counter += 1;

                    if(_verbose)
                    {
                        jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                                           "Plugin {} is running \n",
                                           _plugin_list[i]);
                    }

                }

                if(_plugins_status[i] == "Aborted" || _plugins_status[i] == "InitFailed")
                {
                    if(_verbose)
                    {
                        jhigh().jprint(fmt::fg(fmt::terminal_color::yellow),
                                           "Plugin {} was aborted. It is not possible to start it.\n",
                                           _plugin_list[i]);
                    }
                }

            }

        }

    }
    if(!_start_plugins)
    { // in this case the plugin, when triggered, stops(if not already stopped) all the plugins in the list

        for(int i = 0; i < _n_plugins; i++)
        {

            if(_plugins_status[i] == "Running")
            {// initialized or stopped externally, but not running -> we start the plugin

                if(_triggered)
                {
                    _strt_req_pubs[i]->publish(_stop_command);

                    _strt_res_subs[i]->run(); // processes plugins command service feedback

                }

            }
            else
            { // we enter here if State::Running or State::Aborted

                if(_plugins_status[i] == "Stopped" || _plugins_status[i] == "Initialized")
                {
                    _stopped_plugins_counter += 1;

                    if(_verbose)
                    {
                        jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                                           "Plugin {} is currently stopped.\n",
                                           _plugin_list[i]);
                    }
                }

                if(_plugins_status[i] == "Aborted" || _plugins_status[i] == "InitFailed")
                {
                    if(_verbose)
                    {
                        jhigh().jprint(fmt::fg(fmt::terminal_color::yellow),
                                           "Plugin {} was aborted. It is not possible to stop it.\n",
                                           _plugin_list[i]);
                    }
                }

            }

        }

    }

    _triggered = false; // next run, we'll wait for another trigger signal

    _all_plugins_running = _running_plugins_counter == _plugin_list.size() ? true : false;
    _all_plugins_stopped = _stopped_plugins_counter == _plugin_list.size() ? true : false;

    _plugins_stat_msg.all_plugins_running = _all_plugins_running;
    _plugins_stat_msg.all_plugins_stopped = _all_plugins_stopped;
    _plugins_stat_pub->publish(_plugins_stat_msg);

    _queue.run(); // runs service callbacks

}

void PluginsManagerRt::on_stop()
{
    for(int i = 0; i < _n_plugins; i++)
    {
        _plugins_status[i] = "";
    }

    jinfo("Stopping PluginsManagerRt");
}

void PluginsManagerRt::stopping()
{
    stop_completed();
}

void PluginsManagerRt::on_abort()
{

}

void PluginsManagerRt::on_close()
{
    jinfo("Closing PluginsManagerRt");
}

XBOT2_REGISTER_PLUGIN(PluginsManagerRt, plugins_mngr_rt)
