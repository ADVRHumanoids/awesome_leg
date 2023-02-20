#include "stop_plugins.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

StopPlugins::StopPlugins(const std::string& name) :
    NoThreadAsychActionNode(name, {}),
    Task(name + "_bt_leaf", "")
{

    setRegistrationID(name);

    _asynch_servicepath = _async_service_pattern + "/" + _plugins_manager_name + "/" + _stop_plugins_servname + "/request";

    _plugins_stopper_pub = advertise<awesome_leg::SimpleTriggerRequest>(_asynch_servicepath);

}

NodeStatus StopPlugins::tick()
{

    _plugins_stopper_pub->publish(_trigger);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking StopPlugins action" << Colors::kEndl << std::endl;
    }

    return NodeStatus::RUNNING;

}
