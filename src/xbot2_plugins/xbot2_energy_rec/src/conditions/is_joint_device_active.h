#ifndef IS_JOINT_DEVICE_ACTIVE_H
#define IS_JOINT_DEVICE_ACTIVE_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>

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

            NodeStatus tick() override;

    };

}

#endif // IS_JOINT_DEVICE_ACTIVE_H
