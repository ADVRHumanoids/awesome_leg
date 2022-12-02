#ifndef MAT_REPLAYER_RT
#define MAT_REPLAYER_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include "contact_est.hpp"

#include <awesome_utils/traj_utils.hpp>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <awesome_leg/JumpNow.h>
#include <awesome_leg/MatReplayerStatus.h>

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
using namespace TrajUtils;

typedef Eigen::Array<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * @brief The CartesioEllConfigRt class is a ControlPlugin
 * implementing a trajectory tracking (elliptical traj.). 
 * Depending on the chosen CartesIO config file, the plugin
 * performs either acceleration or impedance control.
 * Moreover, it uses the ros_from_rt synchronization mechanism 
 * to allow runtime configuration of the input trajectory.
 */
class MatReplayerRt : public ControlPlugin
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

    bool _approach_traj_started = false, _approach_traj_finished = false,
        _traj_started = false, _traj_finished = false,
        _imp_traj_started = false, _imp_traj_finished = false,
        _is_first_run = true,
        _pause_started = false, _pause_finished = false,
        _send_pos_ref = true, _send_vel_ref = false,  _send_eff_ref = false,
        _send_pos_ref_backup = true, _send_vel_ref_backup = false,  _send_eff_ref_backup = false,
        _jump = false,
        _compute_approach_traj = true,
        _is_first_jnt_passive = false,
        _resample = false,
        _rt_active, _nrt_exit,
        _jump_now = false, _is_first_trigger = true,
        _is_sim = true,
        _reduce_dumped_sol_size = false,
        _send_whole_traj = false,
        _verbose = false,
        _ft_tip_sensor_found = false,
        _is_drivers_temp_ok = true,
        _is_first_imp_ramp_loop = false;

    int _n_jnts_model,
        _n_jnts_model_ft_est,
        _n_jnts_robot,
        _sample_index = 0,
        _jump_times_vect_size = 1,
        _takeoff_index = -1,
        _jump_phase_state = -1;

    std::string _mat_path, _mat_name, _dump_mat_suffix,
                _urdf_path, _srdf_path,
                _urdf_path_ft_est, _srdf_path_ft_est,
                _tip_link_name, _base_link_name,
                _hw_type,
                _tip_fts_name,
                _contact_linkname = "tip1",
                _test_rig_linkname = "test_rig";

    double _delta_effort_lim,
        _nominal_traj_dt, _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _approach_traj_exec_time = 4.0,
        _approach_traj_time = 0.0,
        _pause_time, _traj_pause_time = 2.0, _approach_traj_pause_time = 5.0,
        _epsi_stiffness = 10, _epsi_damping = 0.1,
        _imp_ramp_time = 0.5, _smooth_imp_time = 0.0,
        _matlogger_buffer_size = 1e4;

    std::vector<int> _contact_dofs{0, 1, 2};

    Eigen::Vector3d _meas_tip_f_loc, _tip_f_est_loc,
                    _meas_tip_t_loc, _tip_t_est_loc,
                    _meas_tip_f_abs, _tip_f_est_abs,
                    _meas_tip_t_abs, _tip_t_est_abs,
                    _base_link_vel, _base_link_omega;

    Eigen::VectorXd _stop_stiffness, _stop_damping,
                    _replay_stiffness, _replay_damping,
                    _meas_stiffness, _meas_damping,
                    _ramp_stiffness, _ramp_damping,
                    _ramp_strt_stiffness, _ramp_strt_damping,
                    _touchdown_damping, _touchdown_stiffness,
                    _stiffness_setpoint, _damping_setpoint,
                    _cntrl_mode,
                    _q_p_meas, _q_p_dot_meas, _tau_meas, _f_cont_meas,
                    _q_p_cmd, _q_p_dot_cmd, _tau_cmd, _f_contact_ref,
                    _q_p_safe_cmd,
                    _traj_time_vector,
                    _effort_lims,
                    _approach_traj_target,
                    _q_p_init_appr_traj, _q_p_trgt_appr_traj,
                    _tip_abs_position,
                    _q_p_ft_est, _q_p_dot_ft_est, _q_p_ddot_ft_est, _tau_ft_est, _f_cont_est,
                    _q_p_dot_ft_est_prev,
                    _meas_driver_temp, _driver_temp_threshold,
                    _auxiliary_vector;

    Eigen::Affine3d _test_rig_pose, _test_rig_pose_inv,
                    _tip_pose_abs, _tip_pose_rel_base_link, _base_link_abs,
                    _base_link_abs_est, _tip_pose_abs_est,
                    _base_link_pos_rel_test_rig;

    Eigen::MatrixXd _q_p_ref, _q_p_dot_ref, _tau_ref, _f_cont_ref;

    PeisekahTrans _peisekah_utils;
    TrajLoader _traj;

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    XBot::ModelInterface::Ptr _model, _model_ft_est;

    std::shared_ptr<XBot::Hal::ForceTorque >_ft_sensor;

    SubscriberPtr<geometry_msgs::PoseStamped> _base_link_pose_sub;
    SubscriberPtr<geometry_msgs::TwistStamped> _base_link_twist_sub;

    ServiceServerPtr<awesome_leg::JumpNowRequest,
                     awesome_leg::JumpNowResponse> _jump_now_srv;

    PublisherPtr<awesome_leg::MatReplayerStatus> _replay_status_pub;

    ContactEstimation::UniquePtr _ft_estimator;

    void get_params_from_config();

    void get_passive_jnt_est(double& pssv_jnt_pos,
                             double& pssv_jnt_vel,
                             double& pssv_jnt_acc);

    void init_model_interface();
    void init_vars();
    void init_cartesio_solver();
    void init_clocks();
    void init_nrt_ros_bridge();
    void init_dump_logger();
    void init_ft_sensor();
    void init_ft_estimator();

    void reset_flags();

    void load_opt_data();
    void resample_trajectory();

    void create_ros_api();

    void ramp_imp_smoothly();

    void saturate_cmds();

    void check_driver_temp_limits();
    
    void update_state();
    void update_state_estimates();

    void update_clocks();

    void set_approach_trajectory();
    void set_trajectory();
    void send_cmds();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void get_abs_tip_position();
    void get_fts_force();

    void is_sim(std::string sim_string);

    bool on_jump_msg_rcvd(const awesome_leg::JumpNowRequest& req,
                          awesome_leg::JumpNowResponse& res);
    void on_base_link_pose_received(const geometry_msgs::PoseStamped& msg);
    void on_base_link_twist_received(const geometry_msgs::TwistStamped& msg);

    void pub_replay_status();

    int was_jump_signal_received();
                 
};

#endif // MAT_REPLAYER_RT
