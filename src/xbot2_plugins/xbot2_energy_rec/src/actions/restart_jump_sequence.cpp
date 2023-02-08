#include "restart_jump_sequence.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

RestartJumpSequence::RestartJumpSequence(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{


}

NodeStatus RestartJumpSequence::tick()
{

    return NodeStatus::RUNNING;

}
