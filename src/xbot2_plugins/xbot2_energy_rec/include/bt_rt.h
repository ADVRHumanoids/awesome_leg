#ifndef BT_RT_H
#define BT_RT_H

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "conditions/custom_conditions.h"
#include "actions/custom_actions.h"

#include <awesome_leg/BtRootStatus.h>
#include <awesome_leg/PluginsManStatus.h>

#include <memory>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace BT;

/**
 * @brief use of BehaviorTreeCPP_v3 inside of a rt plugin
 *
 */

class BtRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;

    void starting() override;

    void run() override;

    void stopping() override;

    void on_stop() override;

    void on_close() override;

    void on_abort() override;

private:

    bool _rt_active, _nrt_exit,
        _is_sim = true, _is_dummy = false, 
        _use_zmq_pub = false, _use_bt_log = false,
        _bt_finished = false,
        _stop_on_completion = false,
        _stop_ticking_root_if_completed = true,
        _verbose = false;

    int _queue_size = 1;

    std::string _mat_path, _dump_mat_suffix,
                _hw_type,
                _bt_description_path,
                _bt_root_topicname = "bt_root",
                _plugin_manager_name = "plugins_mngr_rt",
                _async_service_pattern = "/xbotcore/async_service/xbot_internal/scheduler/",
                _plugins_stat_topicname  = "plugins_manager/plugins_status",
                _tree_logpath = "\tmp",
                _tree_logname = "bt_trace",
                _tree_log_fullpaths;

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e4,
           _recov_energy_thresh = 300 * 5; // [J]

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    awesome_leg::BtRootStatus _bt_root_status_msg;
    PublisherPtr<awesome_leg::BtRootStatus> _bt_root_status_pub;

    NodeStatus _bt_root_status, _bt_root_status_previous;

    BehaviorTreeFactory _factory;

    Tree _tree;

    std::unique_ptr<PublisherZMQ> _zmq_pub_ptr;

    std::unique_ptr<FileLogger> _logger_file;

    // internal publisher and subscribers for triggering the plugin manager
    PublisherPtr<Runnable::Command> _plugins_man_strt_req_pubs;
    SubscriberPtr<bool> _plugins_man_strt_res_subs;

    // internal publisher and subscribers for closing the plugin manager
    PublisherPtr<Runnable::Command> _plugins_man_close_req_pubs;
    SubscriberPtr<bool> _plugins_man_close_res_subs;

//    SubscriberPtr<awesome_leg::PluginsManStatus> _plugins_status_subs;

    void read_config_from_yaml();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void spawn_nrt_thread();

    void init_vars();

    void init_clocks();

    void init_nrt_ros_bridge();

    void init_dump_logger();

    void init_plugin_manager();

    void close_plugin_manager();

    void init_bt();

    std::string bt_status2string(NodeStatus status);

    void pub_bt_status();

    void create_ros_api();

    void update_clocks();

    void add_data2dump_logger();

};

#endif // BT_RT_H
