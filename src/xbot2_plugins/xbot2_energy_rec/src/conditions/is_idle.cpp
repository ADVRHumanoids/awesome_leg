#include "is_idle.h"

#include "../utils_defs.hpp"

using namespace BT;

IsIdle::IsIdle(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto idle_state_callback = [this](const awesome_leg::IdleState& msg)
    {

        _idle_state.idle = msg.idle;

    };

    _idle_state_sub = subscribe<awesome_leg::IdleState>("/" + _idle_pluginname + "/" + _idle_status_topicname,
                            idle_state_callback,
                            _queue_size);

    _idle_state.idle = true; // for safety reasons, we assume to be in idle if not otherwise indicated

};

PortsList IsIdle::providedPorts()
{

  return { OutputPort<bool>("is_idle") };

}

NodeStatus IsIdle::tick()
{

    _idle_state_sub->run();

//    std::cout << Colors::kGreen << "ticking IsIdle" << Colors::kEndl << std::endl;

    NodeStatus result = _idle_state.idle? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("is_idle", _idle_state.idle);

    return result;

}
