#include "always_running.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

AlwaysRunning::AlwaysRunning(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{


}

NodeStatus AlwaysRunning::tick()
{

    return NodeStatus::RUNNING;

}
