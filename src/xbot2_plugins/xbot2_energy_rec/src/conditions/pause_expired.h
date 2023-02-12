#ifndef PAUSE_EXPIRED_H
#define PAUSE_EXPIRED_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/TimerStatus.h>

using namespace XBot;

namespace BT
{

    class PauseExpired : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            PauseExpired(const std::string& name, const NodeConfiguration& config);

            ~PauseExpired() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = true;

            bool _ref_time_set = false;

            int _queue_size = 1;

            std::string _name;

            std::string _timer_status_topicname = "timer";
            std::string _timer_pluginname = "timer_rt";

            double _pause_time = 5.0; // [s]

            double _pause_srt_time = 0.0;

            awesome_leg::TimerStatus _timer_stat_msg;

            SubscriberPtr<awesome_leg::TimerStatus> _timer_status_sub;

            NodeStatus tick() override;

    };

}

#endif // PAUSE_EXPIRED_H
