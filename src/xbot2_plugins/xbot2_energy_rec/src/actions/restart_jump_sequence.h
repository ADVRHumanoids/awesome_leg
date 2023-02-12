#ifndef RESTART_JUMP_SEQUENCE_H
#define RESTART_JUMP_SEQUENCE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <xbot2/journal/journal.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <awesome_leg/RampJntImpRequest.h>
#include <awesome_leg/Go2TakeoffConfigRequest.h>
#include <awesome_leg/PerformTakeoffRequest.h>

using namespace XBot;

namespace BT
{

    class RestartJumpSequence : public AsyncActionNode, public Task
    { // we are working with rt plugins, so we should minimize blocking code
      // that's why we employ asynchronous action nodes

        public:

            RestartJumpSequence(const std::string& name);

        private:

            bool _verbose = true;

            std::string _name;

            int _queue_size = 1;

            std::string _async_service_pattern = "/xbotcore/async_service/";

            std::string _plugin_name = "jmp_replayer_rt";

            std::string _servername1 = "ramp_jnt_imp_srvr";
            std::string _servername2 = "go2takeoff_srvr";
            std::string _servername3 = "perform_takeoff_srvr";

            std::string _asynch_servicepath1,
                        _asynch_servicepath2,
                        _asynch_servicepath3;

            NodeStatus tick() override;

            awesome_leg::RampJntImpRequest _msg1;
            awesome_leg::Go2TakeoffConfigRequest _msg2;
            awesome_leg::PerformTakeoffRequest _msg3;

            PublisherPtr<awesome_leg::RampJntImpRequest> _publisher1;
            PublisherPtr<awesome_leg::Go2TakeoffConfigRequest> _publisher2;
            PublisherPtr<awesome_leg::PerformTakeoffRequest> _publisher3;


    };

}

#endif // RESTART_JUMP_SEQUENCE_H
