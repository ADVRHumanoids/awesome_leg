#ifndef JUMP_REPLAYER_RT
#define JUMP_REPLAYER_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include <awesome_utils/awesome_utils/traj_utils.hpp>

#include <xbot2/ros/ros_support.h>

#include <std_msgs/Bool.h>

#include <awesome_leg/JumpNow.h>
#include <awesome_leg/MatReplayerStatus.h>
#include <awesome_leg/Go2TakeoffConfig.h>
#include <awesome_leg/PerformTakeoff.h>
#include <awesome_leg/RampJntImp.h>
#include <awesome_leg/Go2LandingConfig.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <xbot2/hal/dev_ft.h>

using namespace XBot;
using namespace XBot::Cartesian;
using namespace TrajUtils;

typedef Eigen::Array<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * @brief
 */
class JumpReplayerRt : public ControlPlugin
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
        _send_pos_ref = true, _send_vel_ref = true,  _send_eff_ref = false,
        _is_first_jnt_passive = true,
        _resample = false,
        _is_first_trigger = true,
        _is_sim = true, _is_dummy = false,
        _reduce_dumped_sol_size = false,
        _send_whole_traj = true,
        _verbose = false,
        _is_first_imp_ramp_loop = false,
        _go2takeoff_config = false, _perform_takeoff = false, _ramp_imp = false, _go2landing_config = false,
        _approach_traj_started = false, _traj_started = false, _imp_traj_started = false, _landing_config_started = false,
        _approach_traj_finished = false, _traj_finished = false, _imp_traj_finished = false, _landing_config_reached = false,
        _go2touchdown_config_auto = true,
        _idle = true;

    int _n_jnts_robot,
        _sample_index = 0,
        _takeoff_index = -1,
        _jump_phase_state = -1, 
        _performed_jumps = 0;

    std::string _takeoff_traj_path, _takeoff_traj_name,
                _landing_config_path, _landing_config_name,
                _dump_mat_suffix = "traj_replay",
                _hw_type,
                _dump_path = "/tmp/JumpReplayerRt",
                _landing_stiffness_varname = "stiffness", _landing_damping_varname = "damping",
                _landing_config_varname = "config";

    double _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
        _approach_traj_exec_time = 3.0, _approach_traj_time = 0.0,
        _imp_ramp_time = 4.0, _smooth_imp_time = 0.0,
        _go2takeoff_time = 0.0,
        _go2touchdown_exec_time = 0.5, _go2touchdown_time = 0.0,
        _matlogger_buffer_size = 1e5,
        _resample_err_tolerance = 1e-2,
        _phase = 0.0;

    Eigen::VectorXd _stop_stiffness, _stop_damping,
                    _replay_stiffness, _replay_damping,
                    _meas_stiffness, _meas_damping,
                    _ramp_stiffness, _ramp_damping,
                    _ramp_strt_stiffness, _ramp_strt_damping,
                    _stiffness_setpoint, _damping_setpoint,
                    _q_p_meas, _q_p_dot_meas, _tau_meas, _f_cont_meas,
                    _q_p_cmd, _q_p_dot_cmd, _tau_cmd, _f_contact_ref,
                    _q_p_safe_cmd,
                    _q_p_init_appr_traj, _q_p_trgt_appr_traj,
                    _auxiliary_vector,
                    _jnt_vel_limits, _min_ramp_exec_time;

    Eigen::MatrixXd  _touchdown_damping_aux, _touchdown_stiffness_aux, _landing_config_aux;
    Eigen::VectorXd  _touchdown_damping, _touchdown_stiffness, _landing_config;

    std::vector<double> _q_p_cmd_vect, _q_p_dot_cmd_vect,
                        _tau_cmd_vect,
                        _f_contact_ref_vect;

    Eigen::MatrixXd _q_p_ref, _q_p_dot_ref, _tau_ref, _f_cont_ref;

    PeisekahTrans _peisekah_utils;
    TrajLoader _traj;

    MatLogger2::Ptr _dump_logger;

    MatLogger2::Ptr _landing_config_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    ServiceServerPtr<awesome_leg::PerformTakeoffRequest,
                     awesome_leg::PerformTakeoffResponse> _perform_takeoff_srvr;

    ServiceServerPtr<awesome_leg::Go2TakeoffConfigRequest,
                     awesome_leg::Go2TakeoffConfigResponse> _go2takeoff_config_srvr;

    ServiceServerPtr<awesome_leg::RampJntImpRequest,
                     awesome_leg::RampJntImpResponse> _ramp_jnt_imp_srvr;

    ServiceServerPtr<awesome_leg::Go2LandingConfigRequest,
                     awesome_leg::Go2LandingConfigResponse> _go2lading_config_srvr;

    PublisherPtr<awesome_leg::MatReplayerStatus> _replay_status_pub;

    void get_params_from_config();

    void init_vars();
    void init_clocks();
    void init_ros_bridge();
    void init_dump_logger();

    void reset_clocks();

    void load_opt_data();
    void resample_trajectory();

    void ramp_jnt_impedances();
    void ramp_towards_touchdown_config();

    void saturate_cmds();
    
    void update_state();

    void update_clocks();

    void set_approach_trajectory();
    void set_cmds();
    void send_cmds();

    void add_data2dump_logger();
    void add_data2bedumped();

    void spawn_nrt_thread();

    void is_sim(std::string sim_string);
    void is_dummy(std::string dummy_string);

    void pub_replay_status();

    bool on_perform_takeoff_received(const awesome_leg::PerformTakeoffRequest& req,
                                       awesome_leg::PerformTakeoffResponse& res);
    bool on_go2takeoff_config_received(const awesome_leg::Go2TakeoffConfigRequest& req,
                                     awesome_leg::Go2TakeoffConfigResponse& res);
    bool on_ramp_jnt_imp_received(const awesome_leg::RampJntImpRequest& req,
                                  awesome_leg::RampJntImpResponse& res);
    bool on_go2landing_config_received(const awesome_leg::Go2LandingConfigRequest& req,
                                                  awesome_leg::Go2LandingConfigResponse& res);
                 
};

#endif // JUMP_REPLAYER_RT
