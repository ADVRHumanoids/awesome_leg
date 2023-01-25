#ifndef PLUGINS_STARTER_H
#define PLUGINS_STARTER_H

#include <xbot2/xbot2.h>


using namespace XBot;

/**
 * @brief use of BehaviorTreeCPP_v3 inside of a rt plugin
 *
 */

class PluginsStarterRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void starting() override;

    void run() override;

    void stopping() override;

    void on_stop() override;

    void on_close() override;

    void on_abort() override;

private:

    bool _rt_active, _nrt_exit,
        _is_sim = true,
        _plugins_started = false;

    int _queue_size = 1,
        _n_plugins = 0;

    double _plugin_dt;

    std::vector<std::string> _plugin_list;
    std::vector<bool> _active_plugins;

    std::string _async_service_pattern = "/xbotcore/async_service/xbot_internal/scheduler/";

    // queue object to handle multiple subscribers/servers at once
    std::vector<CallbackQueue> _queues;

    std::vector<std::function<int()>> _callbacks;

    // internal publisher and subscribers
    std::vector<PublisherPtr<Runnable::Command>> _req_pubs;
    std::vector<SubscriberPtr<bool>> _res_subs;

    void read_config_from_yaml();

    void spawn_nrt_thread();

    void init_dump_logger();


};

#endif // PLUGINS_STARTER_H
