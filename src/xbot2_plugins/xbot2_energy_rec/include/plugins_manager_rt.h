#ifndef PLUGINS_MANAGER_H
#define PLUGINS_MANAGER_H

#include <xbot2/xbot2.h>
#include <awesome_leg/PluginsManStatus.h>
#include <xbot2/ros/ros_support.h>
#include <awesome_leg/SimpleTrigger.h>

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
        _is_sim = true, _is_dummy = false,
        _all_plugins_running = false,
        _all_plugins_stopped = false,
        _start_plugins = true,
        _status = false,
        _triggered = false,
        _verbose = false,
        _is_first_trigger = true;

    int _queue_size = 5,
        _queue_size_status = 1,
        _n_plugins = 0,
        _running_plugins_counter = 0,
        _stopped_plugins_counter = 0,
        _retry_counter = 0,
        _retry_n = 2;

    double _plugin_dt;

    std::string _async_service_pattern = "/xbotcore/async_service/xbot_internal/scheduler/",
                _plugins_stat_topicname = "plugins_manager/plugins_status",
                _start_plugins_servname = "start_plugins",
                _stop_plugins_servname = "stop_plugins";

    std::vector<std::string> _plugin_list;
    std::vector<std::string> _plugins_status;
    std::vector<bool> _active_plugins;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    awesome_leg::PluginsManStatus _plugins_stat_msg;

    service::Empty empty_msg;

    // queue object to handle multiple subscribers/servers at once
    std::vector<CallbackQueue> _queues;
    CallbackQueue _queue;

    // internal publisher and subscribers
    std::vector<PublisherPtr<Runnable::Command>> _strt_req_pubs;
    std::vector<SubscriberPtr<bool>> _strt_res_subs;

    std::vector<PublisherPtr<service::Empty>> _status_req_pubs;
    std::vector<SubscriberPtr<std::string>> _status_res_subs;

    // topic to publish global plugin status (i.e. all running or not all running)
    PublisherPtr<awesome_leg::PluginsManStatus> _plugins_stat_pub;

    // server for starting or stopping the plugins externally
    ServiceServerPtr<awesome_leg::SimpleTriggerRequest, awesome_leg::SimpleTriggerResponse> _start_plugins_srvr;
    ServiceServerPtr<awesome_leg::SimpleTriggerRequest, awesome_leg::SimpleTriggerResponse> _stop_plugins_srvr;

    void read_config_from_yaml();

    void spawn_nrt_thread();

    void init_dump_logger();

    bool on_stop_signal(const awesome_leg::SimpleTriggerRequest& req, awesome_leg::SimpleTriggerResponse& res);
    bool on_start_signal(const awesome_leg::SimpleTriggerRequest& req, awesome_leg::SimpleTriggerResponse& res);

    void init_ros_bridge();


};

#endif // PLUGINS_MANAGER_H
