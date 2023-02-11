#include "is_safety_stop.h"

#include "../utils_defs.hpp"

using namespace BT;

IsSafetyStop::IsSafetyStop(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto safety_stop_state_callback = [this](const awesome_leg::SafetyStopState& msg)
    {

        _safety_stop_state.stop = msg.stop;

    };

    _safety_stop_state_sub = subscribe<awesome_leg::SafetyStopState>("/" + _idle_pluginname + "/" + _stop_status_topicname,
                            safety_stop_state_callback,
                            _queue_size);

    _safety_stop_state.stop = false;

};

PortsList IsSafetyStop::providedPorts()
{

  return { OutputPort<bool>("is_safetystop") };

}

NodeStatus IsSafetyStop::tick()
{

    _safety_stop_state_sub->run();

//    std::cout << Colors::kGreen << "ticking IsSafetyStop" << Colors::kEndl << std::endl;

    NodeStatus result = _safety_stop_state.stop? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("is_safetystop", _safety_stop_state.stop);

    return result;

}
