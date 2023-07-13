#ifndef GO2TAKEOFF_CONFIG_H
#define GO2TAKEOFF_CONFIG_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/Go2TakeoffConfigRequest.h>

using namespace XBot;

namespace BT
{

    class Go2TakeoffConfig : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            Go2TakeoffConfig(const std::string& name);

        private:

            bool _verbose = false;

            std::string _name;

            int _queue_size = 1;

            std::string _async_service_pattern = "/xbot/async_service/";

            std::string _plugin_name = "jmp_replayer_rt";

            std::string _servername = "go2takeoff_srvr";

            std::string _asynch_servicepath;

            NodeStatus tick() override;

            using Go2TakeoffConfigRequest = XBot::RpcWrapper<awesome_leg::Go2TakeoffConfigRequest>;
            Go2TakeoffConfigRequest _msg;

            PublisherPtr<Go2TakeoffConfigRequest> _publisher;

    };

}

#endif // GO2TAKEOFF_CONFIG_H
