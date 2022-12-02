#ifndef CONTACT_EST_RT
#define CONTACT_EST_RT

#include <awesome_utils/contact_est_utils.hpp>

#include <awesome_leg/ContactEstStatus.h>

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <thread>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace ContactEstUtils;

/**
 * @brief
 */

class ContactEstRt : public ControlPlugin
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

    bool _is_first_run = true,
        _rt_active, _nrt_exit,
        _is_sim = true,
        _reduce_dumped_sol_size = false,
        _verbose = false,
        _ft_tip_sensor_found = false;

    int _n_jnts_model,
        _n_jnts_model_ft_est,
        _n_jnts_robot;

    std::string _mat_path, _mat_name, _dump_mat_suffix,
                _urdf_path, _srdf_path,
                _urdf_path_ft_est, _srdf_path_ft_est,
                _tip_link_name, _base_link_name,
                _hw_type,
                _tip_fts_name,
                _contact_linkname = "tip1",
                _test_rig_linkname = "test_rig";

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e4;

    std::vector<int> _contact_dofs{0, 1, 2};

    Eigen::Vector3d _meas_tip_f_loc, _tip_f_est_loc,
                    _meas_tip_t_loc, _tip_t_est_loc,
                    _meas_tip_f_abs, _tip_f_est_abs,
                    _meas_tip_t_abs, _tip_t_est_abs,
                    _base_link_vel, _base_link_omega;

    Eigen::VectorXd _q_p_meas, _q_p_dot_meas, _tau_meas, _f_cont_meas,
                    _f_contact_ref,
                    _tip_abs_position,
                    _q_p_ft_est, _q_p_dot_ft_est, _q_p_ddot_ft_est, _tau_ft_est, _f_cont_est,
                    _q_p_dot_ft_est_prev;

    Eigen::Affine3d _test_rig_pose, _test_rig_pose_inv,
                    _tip_pose_abs, _tip_pose_rel_base_link, _base_link_abs,
                    _base_link_abs_est, _tip_pose_abs_est,
                    _base_link_pos_rel_test_rig;

    Eigen::MatrixXd _q_p_ref, _q_p_dot_ref, _tau_ref, _f_cont_ref;

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    XBot::ModelInterface::Ptr _model, _model_ft_est;

    std::shared_ptr<XBot::Hal::ForceTorque >_ft_sensor;

    SubscriberPtr<geometry_msgs::PoseStamped> _base_link_pose_sub;
    SubscriberPtr<geometry_msgs::TwistStamped> _base_link_twist_sub;

//    ContactEstimation::UniquePtr _ft_estimator;

    void get_params_from_config();

    void get_passive_jnt_est(double& pssv_jnt_pos,
                             double& pssv_jnt_vel,
                             double& pssv_jnt_acc);

    void init_model_interface();
    void init_vars();
    void init_clocks();
    void init_nrt_ros_bridge();
    void init_dump_logger();
    void init_ft_sensor();
    void init_ft_estimator();

    void reset_flags();

    void create_ros_api();
    
    void update_state();
    void update_state_estimates();

    void update_clocks();

    void add_data2dump_logger();

    void spawn_nrt_thread();

    void get_abs_tip_position();
    void get_fts_force();

    void is_sim(std::string sim_string);

    void on_base_link_pose_received(const geometry_msgs::PoseStamped& msg);
    void on_base_link_twist_received(const geometry_msgs::TwistStamped& msg);
                 
};

#endif // CONTACT_EST_RT
