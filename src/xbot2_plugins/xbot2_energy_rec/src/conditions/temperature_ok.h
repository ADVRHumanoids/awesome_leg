#ifndef TEMPERATURE_OK_H
#define TEMPERATURE_OK_H

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

    class TemperatureOk : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            TemperatureOk(const std::string& name, const NodeConfiguration& config);

            ~TemperatureOk() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _temp_stat_topicname = "temp_monitor/temp_status";

            std::string _async_service_pattern = "/xbotcore/async_service/";

            std::string _set2idle_servname = "set_cmd_plugins_2idle";

            std::string _asynch_servicepath;

            bool _are_temp_ok = false;

            SubscriberPtr<bool> _temp_stat_subs;

            awesome_leg::IdleState _set2running;

            PublisherPtr<awesome_leg::IdleState> _set_not2idle_pub;

            NodeStatus tick() override;

    };

}

#endif // TEMPERATURE_OK_H
