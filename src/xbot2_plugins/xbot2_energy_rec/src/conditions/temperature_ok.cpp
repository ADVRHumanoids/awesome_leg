#include "temperature_ok.h"
#include "../utils_defs.hpp"

using namespace BT;

TemperatureOk::TemperatureOk(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""){

    setRegistrationID(name);

    // lambda to define callback
    auto temp_status_callback = [this](const awesome_leg::TempOkStatus& msg)
    {
        _are_temp_ok = msg;
    };

    _temp_stat_subs = subscribe<awesome_leg::TempOkStatus>("/" + _temp_monitor_pluginname + "/" + _temp_stat_topicname,
                            temp_status_callback,
                            _queue_size);

    _asynch_servicepath = _async_service_pattern + "/" + _idler_pluginname + "/" + _set2safety_stop_servname + "/request";

    _set_not2stop_pub = advertise<SetSafetyStopRequest>(_asynch_servicepath);

    _release_safety_stop_msg.obj.stop = false;

    _are_temp_ok.drivers_temp_ok = false; // for safety reasons, if we tick the node without having received an update
    // from the callback, we assume temperatures are NOT ok


};

PortsList TemperatureOk::providedPorts()
{

  return { OutputPort<bool>("temperature_ok") };

}

NodeStatus TemperatureOk::tick()
{

    bool are_temp_ok_prev = _are_temp_ok.drivers_temp_ok;

    _temp_stat_subs->run();

    NodeStatus result = _are_temp_ok.drivers_temp_ok? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    if(_verbose)
    {
        std::cout << Colors::kGreen << "ticking TemperatureOk-->temp ok: " << _are_temp_ok << Colors::kEndl << std::endl;
    }

    if(_are_temp_ok.drivers_temp_ok && !are_temp_ok_prev)
    {// we only trigger a state change from idle to running if we previously had a temperature failure (now recovered)

        _set_not2stop_pub->publish(_release_safety_stop_msg);

    }

    setOutput("temperature_ok", _are_temp_ok.drivers_temp_ok);

    return result;

}
