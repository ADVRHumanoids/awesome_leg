#include "plugins_manager_rt.h"

void PluginsManagerRt::read_config_from_yaml()
{
    _plugin_list = getParamOrThrow<std::vector<std::string>>("~plugin_list");

    _queue_size = getParamOrThrow<int>("~queue_size");

    _async_service_pattern = getParamOrThrow<std::string>("~async_service_pattern");
}

bool PluginsManagerRt::on_initialize()
{
    std::string sim_flagname = "sim";

    read_config_from_yaml(); // load params from yaml file

    _plugin_dt = getPeriodSec();

    _n_plugins = _plugin_list.size();

    _strt_req_pubs = std::vector<PublisherPtr<Runnable::Command>>(_n_plugins);
    _strt_res_subs = std::vector<SubscriberPtr<bool>>(_n_plugins);

    _status_req_pubs = std::vector<PublisherPtr<service::Empty>>(_n_plugins);
    _status_res_subs = std::vector<SubscriberPtr<std::string>>(_n_plugins);

    _plugins_status = std::vector<std::string>(_n_plugins);

    for(int i = 0; i < _n_plugins; i++)
    {

        // plugin starting/stopping
        auto res_callback_cmd = [i, this](const bool& msg)
        {

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                               "Received command response {} from plugin: {} \n",
                               msg, _plugin_list[i]);

        };

        _strt_req_pubs[i] = advertise<Runnable::Command>(_async_service_pattern + _plugin_list[i] + "/command/request");
        _strt_res_subs[i] = subscribe<bool>(_async_service_pattern + _plugin_list[i] + "/command/response",
                                    res_callback_cmd,
                                    _queue_size);

        // plugin state getter
        auto res_callback_state = [i, this](const std::string& msg)
        {

            _plugins_status[i] = msg;

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                               "Received status response {} from plugin: {} \n",
                               msg, _plugin_list[i]);

        };

        _status_req_pubs[i] = advertise<service::Empty>(_async_service_pattern + _plugin_list[i] + "/get_state/request");
        _status_res_subs[i] = subscribe<std::string>(_async_service_pattern + _plugin_list[i] + "/get_state/response",
                                    res_callback_state,
                                    _queue_size_status);

        // inizializing knwon plugin state to false
        _plugins_status[i] = "";
    }

    return true;

}

void PluginsManagerRt::starting()
{

    // Move on to run()
    start_completed();

}

void PluginsManagerRt::run()
{

    _plugins_counter = 0;

    if(_start_plugins)
    { // in this case the plugin, when triggered, starts(if not already running) all the plugins in the list

        for(int i = 0; i < _n_plugins; i++)
        {
            _status_req_pubs[i]->publish(empty_msg);
            _status_res_subs[i]->run(); // processes plugins status callbacks

            if(_plugins_status[i] == "Initialized" || _plugins_status[i] == "Stopped")
            {// initialized or stopped externally, but not running -> we start the plugin

                _strt_req_pubs[i]->publish(Command::Start);

                _strt_res_subs[i]->run(); // processes plugins command service feedback

            }
            else
            { // we enter here if State::Running or State::Aborted

                if(_plugins_status[i] == "Running")
                {
                    _plugins_counter += 1;

                    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                                       "Plugin {} successfully started \n",
                                       _plugin_list[i]);
                }

                if(_plugins_status[i] == "Aborted" || _plugins_status[i] == "InitFailed")
                {
                    jhigh().jprint(fmt::fg(fmt::terminal_color::yellow),
                                       "Plugin {} was aborted. It is not possible to start it.\n",
                                       _plugin_list[i]);
                }

            }

        }

    }
    if(!_start_plugins)
    { // in this case the plugin, when triggered, stops(if not already stopped) all the plugins in the list

        for(int i = 0; i < _n_plugins; i++)
        {
            _status_req_pubs[i]->publish(empty_msg);
            _status_res_subs[i]->run(); // processes plugins status callbacks

            if(_plugins_status[i] == "Running")
            {// initialized or stopped externally, but not running -> we start the plugin

                _strt_req_pubs[i]->publish(Command::Stop);

                _strt_res_subs[i]->run(); // processes plugins command service feedback

            }
            else
            { // we enter here if State::Running or State::Aborted

                if(_plugins_status[i] == "Stopped" || _plugins_status[i] == "Initialized")
                {
                    _plugins_counter += 1;

                    jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                                       "Plugin {} successfully stopped.\n",
                                       _plugin_list[i]);
                }

                if(_plugins_status[i] == "Aborted" || _plugins_status[i] == "InitFailed")
                {
                    jhigh().jprint(fmt::fg(fmt::terminal_color::yellow),
                                       "Plugin {} was aborted. It is not possible to stop it.\n",
                                       _plugin_list[i]);
                }

            }

        }

    }

    if(_plugins_counter == _plugin_list.size())
    { // we exit the plugin

          stop();

    }

    //publish plugin status(_all_plugins_running)


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