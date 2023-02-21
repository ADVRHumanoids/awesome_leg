#include "set2safety_stop.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

Set2SafetyStop::Set2SafetyStop(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{

    setRegistrationID(_name);

    _asynch_servicepath = _async_service_pattern + "/" + _idler_pluginname + "/" + _stop_state_servername + "/request";

    _set2stop_pub = advertise<awesome_leg::SetSafetyStopRequest>(_asynch_servicepath);

    _set2stop.stop = true;

}

NodeStatus Set2SafetyStop::tick()
{

    _set2stop_pub->publish(_set2stop);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
