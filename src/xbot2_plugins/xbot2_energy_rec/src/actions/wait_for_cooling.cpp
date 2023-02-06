#include "start_plugins.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

WaitForCooling::WaitForCooling(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", "")
{

    setRegistrationID(name);

    _asynch_servicepath = _async_service_pattern + _start_plugins_servname + "/request";

    _plugins_starter_pub = advertise<service::Empty>(_asynch_servicepath);

}

NodeStatus WaitForCooling::tick()
{

    _wait_for_cooling_pub->publish(empty_msg);

    std::cout << Colors::kBlue << "ticking WaitForCooling action" << Colors::kEndl << std::endl;

    return NodeStatus::RUNNING;

}
