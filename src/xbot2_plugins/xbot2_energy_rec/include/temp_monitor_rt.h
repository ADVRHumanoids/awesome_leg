#ifndef TEMP_MONITOR_RT_H
#define TEMP_MONITOR_RT_H

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <xbot2/hal/dev_ft.h>

#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <awesome_leg/IdleState.h>
#include <awesome_leg/SafetyStopState.h>
#include <awesome_leg/TempOkStatus.h>

using namespace XBot;

class TempMonRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    // initialization method; the plugin won't be run
    // if this returns 'false'
    bool on_initialize() override;

    // callback for the 'Starting' state
    // start_completed() must be called to switch
    // to 'Run' state
    void starting() override;

    // callback for 'Run' state
    void run() override;

    // callback for 'On Stop' state
    void stopping() override;

    // callback for 'On Stop' state
    void on_stop() override;

    // callback for 'On Abort' state
    void on_close() override;

    // callback for 'On Abort' state
    void on_abort() override;


private:

    bool _is_sim = true, _is_dummy = false,
         _verbose = false,
         _is_drivers_temp_ok = false,
         _simulate_temp_if_sim = true,
         _cooling_down = false;

    int _n_jnts_robot, _queue_size = 5;

    std::string _mat_path = "/tmp/", _dump_mat_suffix = "temp_monitor_rt",
                _hw_type,
                _temp_stat_topicname = "temp_monitor/temp_status",
                _idle_status_topicname = "idle_status",
                _safety_stop_topicname = "safety_stop_status",
                _idler_pluginname = "idler_rt";

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e6,
        _driver_temp_threshold = 55.0,
        _driver_temp_threshold_cooling = 45.0,
        _fake_starting_temp = 40.0,
        _temp_rise_rate = 0.0,
        _temp_cooling_rate = 0.0; // deg/s

    std::vector<std::string> _jnt_names;

    Eigen::VectorXd _meas_driver_temp,
                    _driver_temp_thresholds,
                    _driver_temp_thresholds_cooling;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    awesome_leg::TempOkStatus _temp_ok;
    PublisherPtr<awesome_leg::TempOkStatus> _temp_ok_pub;

    awesome_leg::IdleState _idle_status_msg;
    awesome_leg::SafetyStopState _safety_status_msg;

    using IdleState = awesome_leg::IdleState;
    SubscriberPtr<IdleState> _idle_status_sub; // only used to fake temperature in sim
    using SafetyStopState = awesome_leg::SafetyStopState;
    SubscriberPtr<SafetyStopState> _safety_stop_status_sub; // only used to fake temperature in sim

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void init_nrt_ros_bridge();

    void is_sim(std::string sim_string);

    void is_dummy(std::string dummy_string);

    void pub_temp_status();

    void update_state();

    void check_driver_temp_limits();

    void fake_temperature();
}
;

#endif // TEMP_MONITOR_RT_H
