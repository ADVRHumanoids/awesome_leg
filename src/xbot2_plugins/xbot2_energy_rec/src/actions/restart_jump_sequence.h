#ifndef RESTART_JUMP_SEQUENCE_H
#define RESTART_JUMP_SEQUENCE_H

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

    class RestartJumpSequence : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            RestartJumpSequence(const std::string& name);

        private:

            bool _verbose = false;

            std::string _name;

            int _queue_size = 1;

            NodeStatus tick() override;


    };

}

#endif // RESTART_JUMP_SEQUENCE_H
