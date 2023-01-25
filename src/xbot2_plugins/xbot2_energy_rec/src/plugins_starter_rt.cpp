#include "plugins_starter_rt.h"

void PluginsStarterRt::read_config_from_yaml()
{
    _plugin_list = getParamOrThrow<std::vector<std::string>>("~plugin_list");

    _queue_size = getParamOrThrow<int>("~queue_size");

    _async_service_pattern = getParamOrThrow<std::string>("~async_service_pattern");
}

bool PluginsStarterRt::on_initialize()
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
                               "Received response from plugin: {} \n",
                               _plugin_list[i]);

            return 1;
        };

        _req_pubs[i] = advertise<Runnable::Command>(_async_service_pattern + _plugin_list[i] + "/command/request");
        _res_subs[i] = subscribe<bool>(_async_service_pattern + _plugin_list[i] + "/command/response",
                                    res_callback,
                                    _queue_size);

        _active_plugins[i] = false;
    }

    return true;

}

void PluginsStarterRt::starting()
{

    // Move on to run()
    start_completed();

}

void PluginsStarterRt::run()
{

    for(int i = 0; i < _n_plugins; i++)
    {
        _res_subs[i]->run();

        if(!_active_plugins[i])
        {
            _req_pubs[i]->publish(Command::Start);
        }

    }

}

void PluginsStarterRt::on_stop()
{

}

void PluginsStarterRt::stopping()
{
    stop_completed();
}

void PluginsStarterRt::on_abort()
{

}

void PluginsStarterRt::on_close()
{
    jinfo("Closing PluginsStarterRt");
}

XBOT2_REGISTER_PLUGIN(PluginsStarterRt, plugins_strt_rt)
