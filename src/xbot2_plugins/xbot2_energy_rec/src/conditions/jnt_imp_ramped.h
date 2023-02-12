#ifndef JNT_IMP_RAMPED_H
#define JNT_IMP_RAMPED_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/MatReplayerStatus.h>

using namespace XBot;

namespace BT
{

    class JntImpRamped : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            JntImpRamped(const std::string& name, const NodeConfiguration& config);

            ~JntImpRamped() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _plugins_stat_topicname = "replay_status_node";
            std::string _takeoff_pluginname = "jmp_replayer_rt";

            awesome_leg::MatReplayerStatus _jump_status;

            SubscriberPtr<awesome_leg::MatReplayerStatus> _jump_stat_sub;

            NodeStatus tick() override;

    };

}
#endif // JNT_IMP_RAMPED_H
