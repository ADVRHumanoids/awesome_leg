#include "takeoff_performed.h"

#include "../utils_defs.hpp"

using namespace BT;

TakeoffPerformed::TakeoffPerformed(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto jump_status_callback = [this](const awesome_leg::MatReplayerStatus& msg)
    {

        _jump_status.traj_finished = msg.traj_finished;

    };

    _jump_stat_sub = subscribe<awesome_leg::MatReplayerStatus>("/" + _takeoff_pluginname + "/" + _plugins_stat_topicname,
                            jump_status_callback,
                            _queue_size);
};

PortsList TakeoffPerformed::providedPorts()
{

  return { OutputPort<bool>("takeoff_performed") };

}

NodeStatus TakeoffPerformed::tick()
{

    setOutput("takeoff_performed", true);

    _jump_stat_sub->run();

//    std::cout << Colors::kGreen << "ticking TakeoffPerformed" << Colors::kEndl << std::endl;

    NodeStatus result = _jump_status.traj_finished? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    return result;

}
