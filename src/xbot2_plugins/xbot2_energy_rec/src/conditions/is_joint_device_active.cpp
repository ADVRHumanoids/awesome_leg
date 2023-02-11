#include "is_joint_device_active.h"

#include "../utils_defs.hpp"

using namespace BT;

IsJointDevActive::IsJointDevActive(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto dev_info_callback = [this](const Hal::JointMasterInfo& msg)
    {

        _joint_dev_info.mask = msg.mask;

        _is_joint_master_active = _joint_dev_info.mask == 0 ? false : true; // inactive when mask is 0, active otherwise

        if(_verbose)
        {
            jhigh().jprint(fmt::fg(fmt::terminal_color::blue),
                               "Joint master active: {} \n",
                               _is_joint_master_active);
        }


    };

    _joint_master_sub = subscribe<awesome_leg::IdleState>(_joint_dev_info_topicname,
                            dev_info_callback,
                            _queue_size);

    _joint_dev_info.mask = 0; // for safety reasons, we assume joint device to be deactivated if not otherwise indicated

};

PortsList IsJointDevActive::providedPorts()
{

  return { OutputPort<bool>("is_joint_dev_active") };

}

NodeStatus IsJointDevActive::tick()
{

    _joint_master_sub->run();

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking IsJointDevActive" << Colors::kEndl << std::endl;
    }

    NodeStatus result = _is_joint_master_active? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("is_joint_dev_active", _is_joint_master_active);

    return result;

}
