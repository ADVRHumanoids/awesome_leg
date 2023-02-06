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

#include <awesome_leg/SetPlugins2Idle.h>

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
         _is_idle = false;

    int _n_jnts_robot, _queue_size = 1;

    std::string _mat_path, _dump_mat_suffix,
                _hw_type,
                _temp_stat_topicname = "temp_status",
                _idle_status_topicname = "idle_status";

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e6,
        _driver_temp_threshold = 58.0,
        _fake_starting_temp = 40.0,
        _temp_rise_rate = 1.0/60.0,
        _temp_cooling_rate = -5.0/60.0; // deg/s

    std::vector<std::string> _jnt_names;

    Eigen::VectorXd _meas_driver_temp,
                    _driver_temp_thresholds;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    PublisherPtr<bool> _temp_ok_pub;

    awesome_leg::SetPlugins2Idle _idle_status_msg;

    SubscriberPtr<awesome_leg::SetPlugins2Idle> _idle_status_sub; // only used to fake temperature in sim

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void reset_flags();

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
