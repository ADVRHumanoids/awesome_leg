#ifndef STOP_PLUGINS_H
#define STOP_PLUGINS_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/PluginsManStatus.h>
#include <awesome_leg/SimpleTriggerRequest.h>

using namespace XBot;

namespace BT
{

    class StopPlugins : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            StopPlugins(const std::string& name);

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _async_service_pattern = "/xbot/async_service/";

            std::string _plugins_manager_name = "plugins_mngr_rt";

            std::string _stop_plugins_servname = "stop_plugins";

            std::string _asynch_servicepath;

            using SimpleTriggerRequest = XBot::RpcWrapper<awesome_leg::SimpleTriggerRequest>;
            SimpleTriggerRequest _trigger;

            NodeStatus tick() override;

            // internal XBot2 publisher to employ asynchronous server exposed by plugins manager
            PublisherPtr<SimpleTriggerRequest> _plugins_stopper_pub;

    };

}

#endif // STOP_PLUGINS_H
