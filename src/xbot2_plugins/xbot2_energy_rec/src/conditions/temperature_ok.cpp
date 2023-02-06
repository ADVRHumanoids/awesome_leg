#include "temperature_ok.h"
#include "../utils_defs.hpp"

using namespace BT;

TemperatureOk::TemperatureOk(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto temp_status_callback = [this](const bool& msg)
    {
        _are_temp_ok = msg;
    };

    _temp_stat_subs = subscribe<bool>(_temp_stat_topicname,
                            temp_status_callback,
                            _queue_size);

    _asynch_servicepath = _async_service_pattern + _set2idle_servname + "/request";
    _set_not2idle_pub = advertise<awesome_leg::IdleState>(_asynch_servicepath);

    _set2running.idle = false;

};

PortsList TemperatureOk::providedPorts()
{

  return { OutputPort<bool>("temperature_ok") };

}

NodeStatus TemperatureOk::tick()
{

    bool are_temp_ok_prev = _are_temp_ok;

    _temp_stat_subs->run();

    NodeStatus result = _are_temp_ok? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    std::cout << Colors::kGreen << "ticking TemperatureOk-->temp ok: " << _are_temp_ok << Colors::kEndl << std::endl;

    if(_are_temp_ok && !are_temp_ok_prev)
    {// we only trigger a state change from idle to running if we previously had a temperature failure (now recovered)

        _set_not2idle_pub->publish(_set2running);

    }

    return result;

}
