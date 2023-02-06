#ifndef WAIT_FOR_COOLING_H
#define WAIT_FOR_COOLING_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/IdleState.h>

using namespace XBot;

namespace BT
{

    class WaitForCooling : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            WaitForCooling(const std::string& name);

        private:

            bool _verbose = false;

            int _queue_size = 1;

            std::string _async_service_pattern = "/xbotcore/async_service/";

            std::string _start_plugins_servname = "set2idle";

            std::string _asynch_servicepath;

            service::Empty empty_msg;

            NodeStatus tick() override;

            awesome_leg::IdleState _set2idle;
            PublisherPtr<awesome_leg::IdleState> _set2idle_pub;

    };

}

#endif // WAIT_FOR_COOLING_H
