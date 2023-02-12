#include "jnt_imp_ramped.h"

#include "../utils_defs.hpp"

using namespace BT;

JntImpRamped::JntImpRamped(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto jump_status_callback = [this](const awesome_leg::MatReplayerStatus& msg)
    {

        _jump_status.imp_traj_finished = msg.imp_traj_finished;

    };

    _jump_stat_sub = subscribe<awesome_leg::MatReplayerStatus>("/" + _takeoff_pluginname + "/" + _plugins_stat_topicname,
                            jump_status_callback,
                            _queue_size);

    _jump_status.imp_traj_finished = false;
};

PortsList JntImpRamped::providedPorts()
{

  return { OutputPort<bool>("jnt_imp_ramped") };

}

NodeStatus JntImpRamped::tick()
{

    _jump_stat_sub->run();

    if(_verbose)
    {
            std::cout << Colors::kGreen << "ticking TakeoffReached" << Colors::kEndl << std::endl;
    }

    NodeStatus result = _jump_status.imp_traj_finished? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("jnt_imp_ramped", _jump_status.imp_traj_finished);

    return result;

}
