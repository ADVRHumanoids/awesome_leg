#ifndef PERFORM_TAKEOFF_H
#define PERFORM_TAKEOFF_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/PerformTakeoffRequest.h>

using namespace XBot;

namespace BT
{

    class PerformTakeoff : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            PerformTakeoff(const std::string& name);

        private:

            bool _verbose = false;

            std::string _name;

            int _queue_size = 1;

            std::string _async_service_pattern = "/xbot/async_service/";

            std::string _plugin_name = "jmp_replayer_rt";

            std::string _servername = "perform_takeoff_srvr";

            std::string _asynch_servicepath;

            NodeStatus tick() override;

            using PerformTakeoffRequest = XBot::RpcWrapper<awesome_leg::PerformTakeoffRequest>;
            PerformTakeoffRequest _msg;

            PublisherPtr<PerformTakeoffRequest> _publisher;

    };

}

#endif // PERFORM_TAKEOFF_H
