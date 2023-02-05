#include "start_plugins.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

StartPlugins::StartPlugins(const std::string& name) :
    AsyncActionNode(name, {}),
    Task("start_plugins_task", "")
{
    setRegistrationID(name);

    // lambda to define callback
    auto plugins_status_callback = [this](const awesome_leg::PluginsManStatus& msg)
    {

        std::cout << Colors::kBlue << "Received status from plugin manager" << Colors::kEndl << std::endl;

        _plugins_status_msg.all_plugins_running = msg.all_plugins_running;
        _plugins_status_msg.all_plugins_stopped = msg.all_plugins_stopped;

    };

    _plugins_status_subs = subscribe<awesome_leg::PluginsManStatus>(_plugins_stat_topicname,
                            plugins_status_callback,
                            _queue_size);
}

NodeStatus StartPlugins::tick()
{

    std::cout << Colors::kBlue << "ticking StartPlugins action" << Colors::kEndl << std::endl;

    return NodeStatus::RUNNING;

}
