#ifndef MAT_REPLAYER_RT
#define MAT_REPLAYER_RT

#include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

#include "plugin_utils.hpp"

#include <xbot2/ros/ros_support.h>
#include <std_msgs/Bool.h>
#include <awesome_leg/JumpNow.h>

#include <cartesian_interface/ros/RosServerClass.h>

using namespace XBot;
using namespace XBot::Cartesian;

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
    
    std::string _mat_path, _mat_name;

    Eigen::VectorXd _stop_stiffness, _stop_damping, 
                    _cntrl_mode, 
                    _replay_stiffness, _replay_damping, 
                    _q_p_meas, _q_p_dot_meas, 
                    _q_p_cmd, _q_p_dot_cmd, _tau_cmd, 
                    _traj_time_vector, 
                    _effort_lims,
                    _approach_traj_target;

    Eigen::MatrixXd _q_p_ref, _q_p_dot_ref, _tau_ref;

    bool _looped_traj = false, 
        _approach_traj_started = false, _approach_traj_finished = false, 
        _traj_started = false, _traj_finished = false, 
        _first_run = true,  
        _pause_started = false, _pause_finished = false, 
        _send_eff_ref = false,
        _jump = false,
        _recompute_approach_traj = true,
        _is_first_jnt_passive = false;

    double _delta_effort_lim,
        _nominal_traj_dt, _plugin_dt,
        _loop_time = 0.0, _loop_timer_reset_time = 30.0,
        _approach_traj_exec_time = 4.0, 
        _pause_time, _traj_pause_time = 2.0;

    int _n_jnts_model, 
        _sample_index = 0;

    plugin_utils::PeisekahTrans _approach_traj;
    plugin_utils::TrajLoader _traj;

    // XBot::ModelInterface::Ptr _model;

    MatLogger2::Ptr _dump_logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    void get_params_from_config();
    // void init_model_interface();
    void init_clocks();
    void reset_flags();
    void update_clocks();
    void load_opt_data();
    void resample_trajectory();
    void compute_approach_traj();

    void send_approach_trajectory();
    void send_trajectory();
    void saturate_input();
    void update_state();
    void init_dump_logger();
    void add_data2dump_logger();
    void init_nrt_ros_bridge();

    bool on_jump_msg_rcvd(const awesome_leg::JumpNowRequest& req,
                          awesome_leg::JumpNowResponse& res);

    ServiceServerPtr<awesome_leg::JumpNowRequest,
                     awesome_leg::JumpNowResponse> _jump_now_srv;
                 
};

#endif // MAT_REPLAYER_RT