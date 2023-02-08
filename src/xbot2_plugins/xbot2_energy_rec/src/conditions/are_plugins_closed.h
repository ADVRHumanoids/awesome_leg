#ifndef ARE_PLUGINS_CLOSED_H
#define ARE_PLUGINS_CLOSED_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/PluginsManStatus.h>

using namespace XBot;

namespace BT
{

    class ArePluginsClosed : public ConditionNode, public Task
    {
        public:

            // You must provide the function to call when tick() is invoked
            ArePluginsClosed(const std::string& name, const NodeConfiguration& config);

            ~ArePluginsClosed() override = default;

            static PortsList providedPorts(); // this is required to be a static method

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _plugins_stat_topicname = "plugins_manager/plugins_status";

            std::string _plugins_manager_name = "plugins_mngr_rt";

            awesome_leg::PluginsManStatus _plugins_status_msg;

            SubscriberPtr<awesome_leg::PluginsManStatus> _plugins_status_subs;

            NodeStatus tick() override;

    };

}

#endif // ARE_PLUGINS_CLOSED_H
