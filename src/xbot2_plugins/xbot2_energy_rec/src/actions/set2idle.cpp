#include "set2idle.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

Set2Idle::Set2Idle(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{

    setRegistrationID(_name);

    _asynch_servicepath = _async_service_pattern + "/" + _idler_pluginname + "/" + _idle_state_servername + "/request";

    _set2idle_pub = advertise<SetIdleStateRequest>(_asynch_servicepath);

    _set2idle.obj.idle = true;

}

NodeStatus Set2Idle::tick()
{

    _set2idle_pub->publish(_set2idle);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
