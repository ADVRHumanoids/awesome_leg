#include "ramp_jnt_imp.h"

#include <xbot2/intraprocess/topic.h>

#include "../utils_defs.hpp"

using namespace BT;

RampJntImp::RampJntImp(const std::string& name) :
    NoThreadAsychActionNode(name, {}),
    Task(name + "_bt_leaf", ""),
    _name{name}
{

    setRegistrationID(_name);

    _asynch_servicepath = _async_service_pattern + _plugin_name + "/" +_servername + "/request";

    _publisher = advertise<awesome_leg::RampJntImpRequest>(_asynch_servicepath);

    _msg.ramp_imp = true;

}

NodeStatus RampJntImp::tick()
{

    _publisher->publish(_msg);

    if(_verbose)
    {
        std::cout << Colors::kMagenta << "ticking " + _name + " action" << Colors::kEndl << std::endl;

    }

    return NodeStatus::RUNNING;

}
