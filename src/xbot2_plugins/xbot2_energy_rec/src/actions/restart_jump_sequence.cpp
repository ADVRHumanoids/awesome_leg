#include "restart_jump_sequence.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

RestartJumpSequence::RestartJumpSequence(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{
    setRegistrationID(_name);

    _asynch_servicepath1 = _async_service_pattern + _plugin_name + "/" + _servername1 + "/request";
    _asynch_servicepath2 = _async_service_pattern + _plugin_name + "/" + _servername2 + "/request";
    _asynch_servicepath3 = _async_service_pattern + _plugin_name + "/" + _servername3 + "/request";

    _publisher1 = advertise<awesome_leg::RampJntImpRequest>(_asynch_servicepath1);
    _publisher2 = advertise<awesome_leg::Go2TakeoffConfigRequest>(_asynch_servicepath2);
    _publisher3 = advertise<awesome_leg::PerformTakeoffRequest>(_asynch_servicepath3);

    _msg1.ramp_imp = false;
    _msg2.go2takeoff_config = false;
    _msg3.perform_takeoff = false;

}

NodeStatus RestartJumpSequence::tick()
{

    _publisher1->publish(_msg1);
    _publisher2->publish(_msg2);
    _publisher3->publish(_msg3);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
