#ifndef START_PLUGINS_H
#define START_PLUGINS_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/PluginsManStatus.h>

using namespace XBot;

namespace BT
{

    class StartPlugins : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            StartPlugins(const std::string& name);

        private:

            bool _verbose = false;

            int _queue_size = 1;

            NodeStatus tick() override;

            std::string _plugins_stat_topicname = "plugins_manager/plugins_status";

            awesome_leg::PluginsManStatus _plugins_status_msg;

            // internal XBot2 publisher to employ asynchronous server exposed by plugins manager
//            PublisherPtr<> _plugins_starter_pub;

            SubscriberPtr<awesome_leg::PluginsManStatus> _plugins_status_subs;

    };

}

#endif // START_PLUGINS_H
