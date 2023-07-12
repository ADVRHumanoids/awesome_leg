#include "is_joint_device_active.h"

#include "../utils_defs.hpp"

using namespace BT;

IsJointDevActive::IsJointDevActive(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

};

PortsList IsJointDevActive::providedPorts()
{

  return { OutputPort<bool>("is_joint_dev_active") };

}

NodeStatus IsJointDevActive::tick()
{

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking IsJointDevActive" << Colors::kEndl << std::endl;
    }

    _is_joint_master_active = ! Hal::JointSafety::get_shared_safety_flag(); // robot not disabled by xbot2 safety

    NodeStatus result = _is_joint_master_active? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("is_joint_dev_active", _is_joint_master_active);

    return result;

}
