#ifndef CONTACT_EST_RT
#define CONTACT_EST_RT

#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <awesome_utils/awesome_utils/model_interface.hpp>

#include <awesome_leg/ContactEstStatus.h>

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace SignProcUtils;

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
        _ft_tip_sensor_found = false,
        _use_ground_truth_gz = true;

    int _n_jnts_robot,
        _nv_ft_est, _nq_ft_est;

    std::string _mat_path, _mat_name, _dump_mat_suffix,
                _urdf_path_base_est, _srdf_path_base_est,
                _tip_link_name, _base_link_name,
                _hw_type,
                _tip_fts_name,
                _contact_linkname = "tip1",
                _test_rig_linkname = "test_rig";

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e4,
           _ft_est_bw = 10.0,
           _ft_meas_cutoff_freq = 20.0,
           _svd_thresh = 0.05;

    std::vector<int> _selector{0, 1, 2};

    std::vector<std::string> _contacts;

    std::vector<double> _tau_c_vect, _w_c_est_vect,
                        _tip_f_est_abs_vect, _tip_t_est_abs_vect,
                        _f_meas_vect, _w_meas_vect,
                        _meas_tip_f_abs_vect,
                        _meas_tip_t_abs_vect,
                        _meas_tip_f_abs_filt_vect,
                        _meas_tip_t_abs_filt_vect,
                        _tau_c_raw_vect, _CT_v_vect, _g_vect,
                        _p_vect, _p_dot_vect,
                        _q_p_ft_est_vect, _q_p_dot_ft_est_vect,
                        _tau_cmd_vect,
                        _i_c_vect;

    Eigen::VectorXd _base_link_pos,
                    _base_link_vel, _base_link_omega;

    Eigen::VectorXd _q_p_meas, _q_p_dot_meas,
                    _meas_stiff, _meas_damp,
                    _q_p_ref, _q_p_dot_ref,
                    _tau_ff,
                    _tau_cmd,
                    _tau_meas,
                    _tip_abs_position,
                    _q_p_ft_est, _q_p_dot_ft_est, _q_p_ddot_ft_est, _tau_ft_est,
                    _tau_c, _tau_c_raw,
                    _CT_v, _g, _p, _p_dot, _q, _v, _tau,
                    _meas_w_filt;

    Eigen::MatrixXd _C,
                    _K_p, _K_d;

    utils_defs::RotMat3D _R_world_from_tip;

    utils_defs::Wrench _w_c_est, _meas_w_loc, _meas_w_abs,
                  _i_c;

    utils_defs::Force3D _tip_f_est_abs, _meas_tip_f_loc,
                   _meas_tip_f_abs,
                   _meas_tip_f_abs_filt;

    utils_defs::Torque3D _tip_t_est_abs, _meas_tip_t_loc,
                    _meas_tip_t_abs,
                    _meas_tip_t_abs_filt;

    utils_defs::Affine3D _M_world_from_tip, _M_world_from_base_link_est,
                    _M_test_rig_from_world, _M_world_from_test_rig,
                    _M_world_from_base_link, _M_world_from_base_link_ref, _M_base_link_ref_from_base_link,
                    _M_test_rig_from_base_link;

    utils_defs::PosVec3D _base_link_trans_wrt_test_rig;
    utils_defs::LinVel _base_link_vel_wrt_test_rig;

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    std::shared_ptr<XBot::Hal::ForceTorque >_ft_sensor;

    SubscriberPtr<geometry_msgs::PoseStamped> _base_link_pose_sub;
    SubscriberPtr<geometry_msgs::TwistStamped> _base_link_twist_sub;

    PublisherPtr<awesome_leg::ContactEstStatus> _cont_est_status_pub;

    XBot::ModelInterface::Ptr _base_est_model;
    ModelInterface::Model::Ptr _pin_model_ptr;

    XBot::Cartesian::Utils::ForceEstimationMomentumBased::Ptr _fest;
    XBot::ForceTorqueSensor::ConstPtr _fest_virt;

    NumDiff _num_diff_v, _num_diff_p;
    MovAvrgFilt _ft_meas_filt;

    void get_params_from_config();

    void get_passive_jnt_est(double& pssv_jnt_pos,
                             double& pssv_jnt_vel);

    void get_tau_cmd();

    void init_model_interfaces();
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

    void pub_contact_est_status();

};

#endif // CONTACT_EST_RT
