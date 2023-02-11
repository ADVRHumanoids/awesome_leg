#ifndef IS_JOINT_DEVICE_ACTIVE_H
#define IS_JOINT_DEVICE_ACTIVE_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/IdleState.h>

using namespace XBot;

namespace BT
{

    class IsJointDevActive : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            IsJointDevActive(const std::string& name, const NodeConfiguration& config);

            ~IsJointDevActive() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;
            bool _is_joint_master_active = false;

            int _queue_size = 1;

            std::string _joint_dev_info_topicname = "/joint_master/info";

            Hal::JointMasterInfo _joint_dev_info;

            SubscriberPtr<Hal::JointMasterInfo> _joint_master_sub;

            NodeStatus tick() override;

    };

}

#endif // IS_JOINT_DEVICE_ACTIVE_H
