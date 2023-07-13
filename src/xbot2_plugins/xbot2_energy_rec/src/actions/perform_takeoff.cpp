#include "perform_takeoff.h"

#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

PerformTakeoff::PerformTakeoff(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{

    setRegistrationID(_name);

    _asynch_servicepath = _async_service_pattern + _plugin_name + "/" +_servername + "/request";

    _publisher = advertise<PerformTakeoffRequest>(_asynch_servicepath);

    _msg.obj.perform_takeoff = true;

}

NodeStatus PerformTakeoff::tick()
{

    _publisher->publish(_msg);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
