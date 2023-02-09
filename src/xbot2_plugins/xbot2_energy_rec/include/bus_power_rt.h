#ifndef BUS_POWER_RT_H
#define BUS_POWER_RT_H

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <awesome_utils/awesome_utils/calib_utils.hpp>
#include <awesome_utils/awesome_utils/sign_proc_utils.hpp>
#include <awesome_utils/xbot2_utils/xbot2_utils.hpp>
#include <awesome_utils/awesome_utils/power_utils.hpp>

#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

#include <xbot2/ros/ros_support.h>
#include <xbot_msgs/CustomState.h>
#include <xbot_msgs/JointState.h>

#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <awesome_leg/IqEstStatus.h>
#include <awesome_leg/RegPowStatus.h>
#include <awesome_leg/IqMeasStatus.h>

using namespace XBot;
using namespace CalibUtils;
using namespace SignProcUtils;
using namespace Xbot2Utils;
using namespace PowerUtils;

/**
 * @brief The BusPowerRt class is a rt class to validate
 * the power and energy which flows from/to the power bus stage
 * of a system of FOC BLDC actuators
**/

class BusPowerRt : public ControlPlugin
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
         _use_iq_meas = true, 
         _dump_iq_data = false;

    int _n_jnts_robot,
        _der_est_order = 1,
        _iq_calib_window_size = 1000,
        _alpha = 10;

    std::string _mat_path, _dump_mat_suffix,
                _hw_type, 
                _topic_ns = "";

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _matlogger_buffer_size = 1e4,
        _lambda_qp_reg = 1.0,
        _q_dot_3sigma = 0.001;

    double _er = 0.0, _pr = 0.0, _recov_energy = 0.0;

    Eigen::VectorXd _q_p_meas,
                    _q_p_dot_meas, _q_p_dot_meas_filt, _q_p_ddot_est, _q_p_ddot_est_filt,
                    _tau_meas, _tau_meas_filt,
                    _iq_meas, _iq_meas_filt,
                    _K_t, _K_d0, _K_d1, _rot_MoI, _red_ratio,
                    _tau,
                    _iq_est, _iq_friction_torque, _tau_rot_est,
                    _alpha_f0, _alpha_f1,
                    _R, _L_leak, _L_m;

    Eigen::VectorXd _er_k, _pr_k,
                    _pk_joule, _pk_mech, _pk_indct,
                    _ek_joule, _ek_mech, _ek_indct;

    std::vector<double> _iq_est_vect, _q_p_ddot_est_vect, _q_p_ddot_est_filt_vect,
                        _q_p_dot_meas_vect, _q_p_dot_meas_filt_vect,
                        _tau_meas_vect, _tau_meas_filt_vect,
                        _K_t_vect,
                        _K_d0_vect, _K_d1_vect,
                        _rot_MoI_vect, _red_ratio_vect, _iq_friction_torque_vect,
                        _iq_friction_torque_cal_vect,
                        _tau_rot_est_vect,
                        _alpha_f0_vect, _alpha_f1_vect,
                        _K_d0_cal_vect, _K_d1_cal_vect,
                        _iq_meas_vect,
                        _iq_meas_filt_vect;

    std::vector<double> _er_k_vect, _pr_k_vect,
                        _pk_joule_vect, _pk_mech_vect, _pk_indct_vect,
                        _ek_joule_vect, _ek_mech_vect, _ek_indct_vect;

    std::vector<std::string> _jnt_names, _iq_jnt_names;

    MatLogger2::Ptr _dump_logger;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    SubscriberPtr<xbot_msgs::CustomState> _aux_signals_sub;
    SubscriberPtr<xbot_msgs::JointState> _js_signals_sub;

    PublisherPtr<awesome_leg::IqEstStatus> _iq_est_pub;
    PublisherPtr<awesome_leg::RegPowStatus> _reg_pow_pub;
    PublisherPtr<awesome_leg::IqMeasStatus> _iq_meas_pub;

    IqRosGetter::Ptr _iq_getter;
    IqEstimator::Ptr _iq_estimator;

    RegEnergy::Ptr _pow_monitor;

    NumDiff _num_diff;

    MovAvrgFilt _mov_avrg_filter_iq;
    MovAvrgFilt _mov_avrg_filter_tau;
    MovAvrgFilt _mov_avrg_filter_q_dot;

    int _mov_avrg_window_size_iq = 10;
    double _mov_avrg_cutoff_freq_iq = 15.0;
    int _mov_avrg_window_size_tau = 10;
    double _mov_avrg_cutoff_freq_tau = 15.0;
    int _mov_avrg_window_size_iq_meas = 10;
    double _mov_avrg_cutoff_freq_iq_meas = 15.0;
    int _mov_avrg_window_size_q_dot= 10;
    double _mov_avrg_cutoff_freq_q_dot = 15.0;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_dump_logger();

    void reset_flags();

    void update_state();

    void update_clocks();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void init_nrt_ros_bridge();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void pub_iq_est();
    void pub_reg_pow();
    void pub_iq_meas();

    void run_iq_estimation();
    void run_reg_pow_estimation();

};

#endif // BUS_POWER_RT_H
