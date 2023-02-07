#include "stop_plugins.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

StopPlugins::StopPlugins(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", "")
{

    setRegistrationID(name);

    _asynch_servicepath = _async_service_pattern + _stop_plugins_servname + "/request";

    _plugins_stopper_pub = advertise<service::Empty>(_asynch_servicepath);

}

NodeStatus StopPlugins::tick()
{

    _plugins_stopper_pub->publish(empty_msg);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking StopPlugins action" << Colors::kEndl << std::endl;
    }

    return NodeStatus::RUNNING;

}
