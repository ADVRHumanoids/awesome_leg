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

    _req_pubs = std::vector<PublisherPtr<Runnable::Command>>(_n_plugins);
    _res_subs = std::vector<SubscriberPtr<bool>>(_n_plugins);

    _active_plugins = std::vector<bool>(_n_plugins);

    for(int i = 0; i < _n_plugins; i++)
    {
        auto res_callback = [i, this](const bool& msg)
        {
            _active_plugins[i] = msg;

            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                               "Received response {} from plugin: {} \n",
                               msg, _plugin_list[i]);

        };

        _req_pubs[i] = advertise<Runnable::Command>(_async_service_pattern + _plugin_list[i] + "/command/request");
        _res_subs[i] = subscribe<bool>(_async_service_pattern + _plugin_list[i] + "/command/response",
                                    res_callback,
                                    _queue_size);

        _active_plugins[i] = false;
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

    _active_plugins_counter = 0;

    for(int i = 0; i < _n_plugins; i++)
    {
        _res_subs[i]->run();

        if(!_active_plugins[i])
        {
            _req_pubs[i]->publish(Command::Start);
        }
        else
        {
            _active_plugins_counter += 1;

            jhigh().jprint(fmt::fg(fmt::terminal_color::green),
                               "Plugin {} successfully started \n",
                               _plugin_list[i]);
        }

    }

    if(_active_plugins_counter == _plugin_list.size())
    { // we exit the plugin

        stop();

    }

}

void PluginsManagerRt::on_stop()
{
    for(int i = 0; i < _n_plugins; i++)
    {
        _active_plugins[i] = false;
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
