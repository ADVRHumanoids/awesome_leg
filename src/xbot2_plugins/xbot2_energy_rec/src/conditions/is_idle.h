#ifndef IS_IDLE_H
#define IS_IDLE_H

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

    class IsIdle : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            IsIdle(const std::string& name, const NodeConfiguration& config);

            ~IsIdle() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _idle_status_topicname = "idle_status";
            std::string _idle_pluginname = "idler_rt";

            awesome_leg::IdleState _idle_state;

            SubscriberPtr<awesome_leg::IdleState> _idle_state_sub;

            NodeStatus tick() override;

    };

}

#endif // IS_IDLE_H
