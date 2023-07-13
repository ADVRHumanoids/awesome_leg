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
#include <awesome_leg/PerformTakeoffRequest.h>
#include <awesome_leg/Go2LandingConfig.h>

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

            std::string _async_service_pattern = "/xbot/async_service/";

            std::string _plugin_name = "jmp_replayer_rt";

            std::string _servername1 = "ramp_jnt_imp_srvr";
            std::string _servername2 = "go2takeoff_srvr";
            std::string _servername3 = "perform_takeoff_srvr";
            std::string _servername4 = "go2touchdown_config_srvr";

            std::string _asynch_servicepath1,
                        _asynch_servicepath2,
                        _asynch_servicepath3,
                        _asynch_servicepath4;

            NodeStatus tick() override;

            using RampJntImpRequest = XBot::RpcWrapper<awesome_leg::RampJntImpRequest>;
            using Go2TakeoffConfigRequest = XBot::RpcWrapper<awesome_leg::Go2TakeoffConfigRequest>;
            using PerformTakeoffRequest = XBot::RpcWrapper<awesome_leg::PerformTakeoffRequest>;
            using Go2LandingConfigRequest = XBot::RpcWrapper<awesome_leg::Go2LandingConfigRequest>;

            RampJntImpRequest _msg1;
            Go2TakeoffConfigRequest _msg2;
            PerformTakeoffRequest _msg3;
            Go2LandingConfigRequest _msg4;

            PublisherPtr<RampJntImpRequest> _publisher1;
            PublisherPtr<Go2TakeoffConfigRequest> _publisher2;
            PublisherPtr<PerformTakeoffRequest> _publisher3;
            PublisherPtr<Go2LandingConfigRequest> _publisher4;

    };

}

#endif // RESTART_JUMP_SEQUENCE_H
