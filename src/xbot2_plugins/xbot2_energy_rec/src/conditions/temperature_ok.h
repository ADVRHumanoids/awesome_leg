#ifndef TEMPERATURE_OK_H
#define TEMPERATURE_OK_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/SetSafetyStopRequest.h>
#include <awesome_leg/TempOkStatus.h>

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

            std::string _async_service_pattern = "/xbot/async_service/";

            std::string _idler_pluginname = "idler_rt";

            std::string _temp_monitor_pluginname = "temp_monitor_rt";

            std::string _set2safety_stop_servname = "set_cmd_plugins_2safetystop";

            std::string _asynch_servicepath;

            awesome_leg::TempOkStatus _are_temp_ok;

            SubscriberPtr<awesome_leg::TempOkStatus> _temp_stat_subs;

            using SetSafetyStopRequest = XBot::RpcWrapper<awesome_leg::SetSafetyStopRequest>;
            SetSafetyStopRequest _release_safety_stop_msg;

            PublisherPtr<SetSafetyStopRequest> _set_not2stop_pub;

            NodeStatus tick() override;

    };

}

#endif // TEMPERATURE_OK_H
