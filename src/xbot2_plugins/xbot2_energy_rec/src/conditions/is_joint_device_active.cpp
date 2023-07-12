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

    std::shared_ptr<std::atomic_bool> _is_joint_master_active_ptr = Hal::JointSafety::get_shared_safety_flag(); // robot not disabled by xbot2 safety

    _is_joint_master_active = ! *_is_joint_master_active_ptr.get();

    NodeStatus result = _is_joint_master_active? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    setOutput("is_joint_dev_active", _is_joint_master_active);

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking IsJointDevActive" << Colors::kEndl << std::endl;
        std::cout << Colors::kGreen << _is_joint_master_active << Colors::kEndl << std::endl;
    }

    return result;

}
