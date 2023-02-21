#include "go2takeoff_config.h"

#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

Go2TakeoffConfig::Go2TakeoffConfig(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{

    setRegistrationID(_name);

    _asynch_servicepath = _async_service_pattern + _plugin_name + "/" +_servername + "/request";

    _publisher = advertise<awesome_leg::Go2TakeoffConfigRequest>(_asynch_servicepath);

    _msg.go2takeoff_config = true;

}

NodeStatus Go2TakeoffConfig::tick()
{

    _publisher->publish(_msg);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
