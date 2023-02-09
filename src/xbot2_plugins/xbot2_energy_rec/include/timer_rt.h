// main XBot2 include
#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>
#include <awesome_leg/TimerStatus.h>
#include <awesome_leg/SimpleTrigger.h>

using namespace XBot;

/**
 * @brief The TimerRt class is a ControlPlugin
 * to broadcast a high resolution timer to both ros and internal topics, for
 * use by other plugins
 */
class TimerRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void starting() override;

    void run() override;

    void on_stop() override;

    void stopping() override;

    void on_close() override;

private:

    bool _reset_timer = false;

    std::string _timer_plugin_topicname = "timer";

    std::string _reset_timers_servname = "reset_timers";

    double _elapsed_st_time = 0.0,
           _elapsed_wall_time = 0.0,
           _elapsed_sys_time = 0.0,
           _elapsed_hr_time = 0.0;

    chrono::steady_clock::time_point _st_time;
    chrono::wall_clock::time_point _wall_time;
    chrono::system_clock::time_point _sys_time;
    chrono::high_resolution_clock::time_point _hr_time;

    RosSupport::UniquePtr _ros;
    CallbackQueue _queue;

    awesome_leg::TimerStatus _timer_msg;
    PublisherPtr<awesome_leg::TimerStatus> _timer_pub;

    ServiceServerPtr<awesome_leg::SimpleTriggerRequest, awesome_leg::SimpleTriggerResponse> _reset_timers_srv;

    bool on_reset_sign_received(const awesome_leg::SimpleTriggerRequest& req, awesome_leg::SimpleTriggerResponse& res);

};

