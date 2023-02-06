#include "wait_for_cooling.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

WaitForCooling::WaitForCooling(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", "")
{

    setRegistrationID(name);

    _asynch_servicepath = _async_service_pattern + _idle_state_servername + "/request";

    _set2idle_pub = advertise<awesome_leg::IdleState>(_asynch_servicepath);

    _set2idle.idle = true;

}

NodeStatus WaitForCooling::tick()
{

    _set2idle_pub->publish(_set2idle);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking WaitForCooling action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
