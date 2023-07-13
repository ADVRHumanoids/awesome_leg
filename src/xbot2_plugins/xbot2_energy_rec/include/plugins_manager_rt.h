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
        _triggered = true,
        _verbose = false,
        _is_first_trigger = true;

    int _queue_size = 5,
        _queue_size_status = 5,
        _n_plugins = 0,
        _running_plugins_counter = 0,
        _stopped_plugins_counter = 0,
        _retry_counter = 0,
        _retry_n = 2;

    double _plugin_dt;

    std::string _async_service_pattern = "/xbot/async_service/xbot/task/",
                _plugins_stat_topicname = "plugins_manager/plugins_status",
                _start_plugins_servname = "start_plugins",
                _stop_plugins_servname = "stop_plugins";

    std::vector<std::string> _plugin_list;
    std::vector<std::string> _plugins_status;
    std::vector<bool> _active_plugins;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    awesome_leg::PluginsManStatus _plugins_stat_msg;

    XBot::RpcWrapper<service::Empty> _empty_msg;

    // queue object to handle multiple subscribers/servers at once
    std::vector<CallbackQueue> _queues;
    CallbackQueue _queue;

    // internal publisher and subscribers
    std::vector<PublisherPtr< XBot::RpcWrapper<XBot::Runnable::Command>>> _strt_req_pubs;
    std::vector<SubscriberPtr<XBot::RpcWrapper<bool>>> _strt_res_subs;

    std::vector<PublisherPtr<XBot::RpcWrapper<service::Empty>>> _status_req_pubs;
    std::vector<SubscriberPtr<XBot::RpcWrapper<std::string>>> _status_res_subs;

    // topic to publish global plugin status (i.e. all running or not all running)
    PublisherPtr<awesome_leg::PluginsManStatus> _plugins_stat_pub;

//    using SimpleTriggerRequest = XBot::RpcWrapper<awesome_leg::SimpleTriggerRequest>;
//    using SimpleTriggerResponse = XBot::RpcWrapper<awesome_leg::SimpleTriggerResponse>;
    using SimpleTriggerRequest = awesome_leg::SimpleTriggerRequest;
    using SimpleTriggerResponse = awesome_leg::SimpleTriggerResponse;

    // async server for starting or stopping the plugins externally
    ServiceServerPtr<SimpleTriggerRequest, SimpleTriggerResponse> _start_plugins_srvr;
    ServiceServerPtr<SimpleTriggerRequest, SimpleTriggerResponse> _stop_plugins_srvr;

    XBot::RpcWrapper<Runnable::Command> _start_command;
    XBot::RpcWrapper<Runnable::Command> _stop_command;

    void read_config_from_yaml();

    void spawn_nrt_thread();

    void init_dump_logger();

    bool on_stop_signal(const SimpleTriggerRequest& req, SimpleTriggerResponse& res);
    bool on_start_signal(const SimpleTriggerRequest& req, SimpleTriggerResponse& res);

    void init_ros_bridge();


};

#endif // PLUGINS_MANAGER_H
