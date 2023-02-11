#include "takeoff_reached.h"

#include "../utils_defs.hpp"

using namespace BT;

TakeoffReached::TakeoffReached(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto jump_status_callback = [this](const awesome_leg::MatReplayerStatus& msg)
    {

        _jump_status.approach_traj_finished = msg.approach_traj_finished;

    };

    _jump_stat_sub = subscribe<awesome_leg::MatReplayerStatus>("/" + _takeoff_pluginname + "/" + _plugins_stat_topicname,
                            jump_status_callback,
                            _queue_size);
};

PortsList TakeoffReached::providedPorts()
{

  return { OutputPort<bool>("takeoff_reached") };

}

NodeStatus TakeoffReached::tick()
{

    _jump_stat_sub->run();

//    std::cout << Colors::kGreen << "ticking TakeoffReached" << Colors::kEndl << std::endl;

    NodeStatus result = _jump_status.approach_traj_finished? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("takeoff_reached", _jump_status.approach_traj_finished);

    return result;

}
