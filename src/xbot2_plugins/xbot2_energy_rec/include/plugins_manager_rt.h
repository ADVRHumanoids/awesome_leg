#ifndef PLUGINS_MANAGER_H
#define PLUGINS_MANAGER_H

#include <xbot2/xbot2.h>


using namespace XBot;

/**
 * @brief use of BehaviorTreeCPP_v3 inside of a rt plugin
 *
 */

class PluginsManagerRt : public ControlPlugin
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
        _all_plugins_running = false,
        _start_plugins = false;

    int _queue_size = 1,
        _queue_size_status = 1,
        _n_plugins = 0,
        _plugins_counter = 0;

    double _plugin_dt;

    std::string _async_service_pattern = "/xbotcore/async_service/xbot_internal/scheduler/";

    std::vector<std::string> _plugin_list;
    std::vector<std::string> _plugins_status;
    std::vector<bool> _active_plugins;

    service::Empty empty_msg;

    // queue object to handle multiple subscribers/servers at once
    std::vector<CallbackQueue> _queues;

    // internal publisher and subscribers
    std::vector<PublisherPtr<Runnable::Command>> _strt_req_pubs;
    std::vector<SubscriberPtr<bool>> _strt_res_subs;

    std::vector<PublisherPtr<service::Empty>> _status_req_pubs;
    std::vector<SubscriberPtr<std::string>> _status_res_subs;

    void read_config_from_yaml();

    void spawn_nrt_thread();

    void init_dump_logger();


};

#endif // PLUGINS_MANAGER_H
