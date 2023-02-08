#include "are_plugins_closed.h"
#include "../utils_defs.hpp"

using namespace BT;

ArePluginsClosed::ArePluginsClosed(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto plugins_status_callback = [this](const awesome_leg::PluginsManStatus& msg)
    {

        _plugins_status_msg.all_plugins_running = msg.all_plugins_running;
        _plugins_status_msg.all_plugins_stopped = msg.all_plugins_stopped;

    };

    _plugins_status_subs = subscribe<awesome_leg::PluginsManStatus>("/" + _plugins_manager_name + "/" + _plugins_stat_topicname,
                            plugins_status_callback,
                            _queue_size);
};

PortsList ArePluginsClosed::providedPorts()
{

  return { OutputPort<bool>("are_plugins_closed") };

}

NodeStatus ArePluginsClosed::tick()
{

    setOutput("are_plugins_closed", true);

    _plugins_status_subs->run();

//    std::cout << Colors::kGreen << "ticking ArePluginsRunning" << Colors::kEndl << std::endl;

    NodeStatus result = _plugins_status_msg.all_plugins_stopped? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    return result;

}
