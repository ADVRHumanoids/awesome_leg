#include "pause_expired.h"

#include "../utils_defs.hpp"

using namespace BT;

PauseExpired::PauseExpired(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config),
    Task(name + "_bt_leaf", ""),
    _name{name}

{

    setRegistrationID(name);

    // lambda to define callback
    auto timer_status_callback = [this](const awesome_leg::TimerStatus& msg)
    {

        if(!_ref_time_set)
        { // we reset the reference time
            _pause_srt_time = msg.hr_time;

            _ref_time_set = true;
        }

        _timer_stat_msg.hr_time = msg.hr_time - _pause_srt_time;

    };

    _timer_status_sub = subscribe<awesome_leg::TimerStatus>("/" + _timer_pluginname + "/" + _timer_status_topicname,
                            timer_status_callback,
                            _queue_size);
};

PortsList PauseExpired::providedPorts()
{

  return { OutputPort<bool>("pause_expired") };

}

NodeStatus PauseExpired::tick()
{

    setOutput("pause_expired", true);

    _timer_status_sub->run();

    std::cout << Colors::kGreen << "ticking " + _name  << " " << _timer_stat_msg.hr_time <<  Colors::kEndl << std::endl;

    NodeStatus result = _timer_stat_msg.hr_time >= _pause_time? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    if(result == NodeStatus::SUCCESS)
    { // next time this condition is triggered, we reset the internal reference time

        _ref_time_set = false;

    }

    return result;

}
