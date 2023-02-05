#include "start_plugins.h"
#include "../utils_defs.hpp"

#include <xbot2/intraprocess/topic.h>

using namespace BT;

StartPlugins::StartPlugins(const std::string& name) :
    AsyncActionNode(name, {}),
    Task(name + "_bt_leaf", "")
{

    setRegistrationID(name);

    _asynch_servicepath = _async_service_pattern + _plugins_manager_name + "/" + _start_plugins_servname + "/request";

    _plugins_starter_pub = advertise<service::Empty>(_asynch_servicepath);

}

NodeStatus StartPlugins::tick()
{

    _plugins_starter_pub->publish(empty_msg);

    std::cout << Colors::kBlue << "ticking StartPlugins action" << Colors::kEndl << std::endl;

    return NodeStatus::RUNNING;

}
