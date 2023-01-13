#ifndef BASE_EST_RT
#define BASE_EST_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>
#include <xbot2/hal/dev_ft.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <thread>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base_estimation/base_estimation.h>
#include <base_estimation/ContactsStatus.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <base_estimation/contact_viz.h>

#include <awesome_utils/typedefs.hpp>
#include <awesome_utils/sign_proc_utils.hpp>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace ikbe;
using namespace SignProcUtils;

/**
 * @brief XBot2 real time plugin to perform base estimation
 *
 */

class BaseEstRt : public ControlPlugin
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

    bool _rt_active, _nrt_exit,
        _is_sim = true,
        _ft_tip_sensor_found = false;

    int _n_jnts_robot,
        _nv_base_est, _nq_base_est;

    std::string _mat_path, _dump_mat_suffix,
                _srdf_path_base_est,
                _urdf_path_base_est,
                _tip_link_name, _base_link_name, _test_rig_linkname = "test_rig",
                _hw_type,
                _tip_fts_name,
                _contact_linkname = "tip1",
                _ik_problem_path;

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e4,
           _mov_avrg_cutoff_freq = 15.0;

    utils_defs::Force3D _meas_tip_f_loc,
                   _meas_tip_f_abs;

    utils_defs::Torque3D _meas_tip_t_loc,
                   _meas_tip_t_abs;

    utils_defs::Wrench _meas_w_loc, _meas_w_abs;

    Eigen::VectorXd _q_p_meas, _q_p_dot_meas,
                    _tau_meas, _q_p_ref, _q_p_dot_ref,
                    _tau_ff, _meas_stiff, _meas_damp,
                    _tau_cmd,
                    _meas_w_filt,
                    _base_link_vel, _base_link_omega;

    Eigen::MatrixXd _K_p, _K_d;

    utils_defs::RotMat3D _R_world_from_tip;

    utils_defs::Affine3D _M_world_from_tip, _M_world_from_base_link_est,
                    _M_test_rig_from_world, _M_world_from_test_rig,
                    _M_world_from_base_link, _M_world_from_base_link_ref, _M_base_link_ref_from_base_link,
                    _M_test_rig_from_base_link;

    MovAvrgFilt _ft_meas_filt;

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    std::shared_ptr<XBot::Hal::ForceTorque >_ft_sensor;

    SubscriberPtr<geometry_msgs::PoseStamped> _base_link_pose_sub;
    SubscriberPtr<geometry_msgs::TwistStamped> _base_link_twist_sub;

    XBot::ModelInterface::Ptr _base_est_model;
    BaseEstimation::UniquePtr _est;
    BaseEstimation::Options _be_options;
    std::map<std::string, ForceTorqueSensor::ConstPtr> _ft_map;

    void get_params_from_config();
    void is_sim(std::string sim_string);
    void reset_flags();

    void spawn_nrt_thread();

    void init_model_interfaces();
    void init_vars();
    void init_clocks();
    void init_nrt_ros_bridge();
    void init_dump_logger();
    void init_base_estimator();
    void init_ft_sensor();

    void create_ros_api();

    void update_clocks();
    void update_state();

    void add_data2dump_logger();

    void get_tau_cmd();
    void get_fts_force();

    void on_base_link_pose_received(const geometry_msgs::PoseStamped& msg);
    void on_base_link_twist_received(const geometry_msgs::TwistStamped& msg);

};

#endif // BASE_EST_RT
