#ifndef IMPACT_DETECTOR_RT_H
#define IMPACT_DETECTOR_RT_H

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <thread>

#include <awesome_leg/BaseEstStatus.h>
#include <awesome_leg/SetRegEnergyMonitoringStatusRequest.h>
#include <awesome_leg/MatReplayerStatus.h>
#include <awesome_leg/ImpactStatus.h>

using namespace XBot;

class ImpactDetectorRt : public ControlPlugin
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
        _use_gz_ground_truth = false;

    int _queue_size = 1;

    std::string _mat_path, _dump_mat_suffix,
                _hw_type,
                _base_est_status_topicname = "base_estimation_node",
                _base_est_pluginname = "base_est_rt",
                _jump_replay_topicname = "replay_status_node",
                _jump_replay_pluginname = "jmp_replayer_rt",
                _pow_monitor_pluginname = "bus_power_rt_iq_model",
                _pow_monitor_servicename = "start_rec_energy_monitor",
                _async_service_pattern = "/xbotcore/async_service/",
                _impact_state_topicname = "impact_state",
                _asynch_servicepath;

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e6;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    awesome_leg::BaseEstStatus _base_est_status, _base_est_status_prev;
    awesome_leg::SetRegEnergyMonitoringStatusRequest _start_monitoring_msg;
    awesome_leg::SetRegEnergyMonitoringStatusRequest _stop_monitoring_msg;
    awesome_leg::ImpactStatus _impact_status_msg;

    awesome_leg::MatReplayerStatus _jump_replay_status;

    PublisherPtr<awesome_leg::SetRegEnergyMonitoringStatusRequest> _monitoring_pow_switch_pub;
    PublisherPtr<awesome_leg::ImpactStatus> _impact_status_pub;

    SubscriberPtr<awesome_leg::MatReplayerStatus> _jump_replay_status_sub;
    SubscriberPtr<awesome_leg::BaseEstStatus> _base_est_status_sub;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void reset_flags();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void init_communications();

    void is_sim(std::string sim_string);

    void is_dummy(std::string dummy_string);

    void trigger_reg_pow_monitor();
}
;

#endif // IMPACT_DETECTOR_RT_H
