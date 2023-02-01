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

#include "conditions/are_plugins_running.h"
#include "actions/start_plugins.h"

#include <awesome_leg/BtRootStatus.h>

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
        _is_sim = true;

    std::string _mat_path, _dump_mat_suffix,
                _hw_type,
                _bt_description_path,
                _bt_root_topicname = "bt_root";

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e4;

    std::vector<std::string> _plugin_list;
    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    awesome_leg::BtRootStatus _bt_root_status_msg;

    PublisherPtr<awesome_leg::BtRootStatus> _bt_root_status_pub;

    NodeStatus _bt_root_status;

    BehaviorTreeFactory _factory;

    Tree _tree;

    PublisherZMQ _publisher_zmq(Tree);

    void read_config_from_yaml();

    void is_sim(std::string sim_string);

    void spawn_nrt_thread();

    void init_vars();

    void init_clocks();

    void init_nrt_ros_bridge();

    void init_dump_logger();

    void init_bt();

    std::string bt_status2string(NodeStatus status);

    void pub_bt_status();

    void create_ros_api();

    void update_clocks();

    void add_data2dump_logger();

};

#endif // BT_RT_H
