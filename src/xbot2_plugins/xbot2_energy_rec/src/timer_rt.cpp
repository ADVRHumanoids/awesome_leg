#include "timer_rt.h"
#include "fmt/chrono.h"

using namespace std::chrono_literals;

bool TimerRt::on_initialize()
{
    _timer_plugin_topicname = getParamOrThrow<std::string>("~timer_plugin_topicname");
    _reset_timers_servname = getParamOrThrow<std::string>("~reset_timers_servname");

    _timer_msg.wall_time = 0.0;
    _timer_msg.st_time = 0.0;
    _timer_msg.sys_time = 0.0;
    _timer_msg.hr_time = 0.0;

    ros::NodeHandle nh(getName());

    _ros = std::make_unique<RosSupport>(nh);

    _timer_pub = _ros->advertise<awesome_leg::TimerStatus>(_timer_plugin_topicname.c_str(), 1, _timer_msg);

    _reset_timers_srv = _ros->advertiseService(_reset_timers_servname,
                                                &TimerRt::on_reset_sign_received,
                                                this,
                                                &_queue);
    return true;
}

bool TimerRt::on_reset_sign_received(const awesome_leg::SimpleTriggerRequest& req, awesome_leg::SimpleTriggerResponse& res)
{

    _reset_timer = true;

    res.success = _reset_timer;

    return res.success;
}


void TimerRt::starting()
{
    _wall_time = chrono::wall_clock::now();

    // high_resolution_clock is the system most precise
    // steady clock, and it is useful for profiling
    _hr_time = chrono::high_resolution_clock::now();

    // steady clock is used for measuring time intervals
    // (e.g. periodic tasks, timeouts, etc)
    // uses simulation time if available, otherwise
    // it is the same as high_resolution_clock
    _st_time = chrono::steady_clock::now();

    // system clock is used for timestamps that must be
    // synchronized across multiple machines
    // uses simulation time if available, otherwise
    // it is the same as wall_clock
    _sys_time = chrono::system_clock::now();

    start_completed();
}

void TimerRt::run()
{
    using namespace std::chrono;

    if(_reset_timer)
    { // we reset the reference time of the clocks
        _st_time = chrono::steady_clock::now();
        _sys_time = chrono::system_clock::now();
        _wall_time = chrono::wall_clock::now();
        _hr_time = chrono::high_resolution_clock::now();

        _reset_timer = false; // we'll wait for the next reset signal

    }

    auto st_elapsed = chrono::steady_clock::now() - _st_time;
    auto sys_elapsed = chrono::system_clock::now() - _sys_time;
    auto wall_elapsed = chrono::wall_clock::now() - _wall_time;
    auto hr_elapsed = chrono::high_resolution_clock::now() - _hr_time;

    _timer_msg.wall_time = duration<double>(st_elapsed).count();
    _timer_msg.st_time = duration<double>(wall_elapsed).count();
    _timer_msg.sys_time = duration<double>(sys_elapsed).count();
    _timer_msg.hr_time = duration<double>(hr_elapsed).count();

    _timer_pub->publish(_timer_msg);
    _queue.run();
}

void TimerRt::on_stop()
{

}

void TimerRt::stopping()
{
    stop_completed();
}

void TimerRt::on_close()
{

    _reset_timer = true; // next stime the plugin is started
    // we reset the durations

    jinfo("Closing TimerRt");
}

XBOT2_REGISTER_PLUGIN(TimerRt, timer_rt)
