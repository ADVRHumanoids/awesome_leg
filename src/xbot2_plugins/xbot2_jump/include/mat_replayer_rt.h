#ifndef MAT_REPLAYER_RT
#define MAT_REPLAYER_RT

#include <matlogger2/matlogger2.h>
#include <xbot2/xbot2.h>
#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <thread>

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
    
    std::string _urdf_path, _srdf_path, _mat_path;

    Eigen::VectorXd _stop_stiffness, _stop_damping, 
                    _cntrl_mode, 
                    _replay_stiffness, _replay_damping, 
                    _q_p_meas, 
                    _q_p_cmd, _q_p_dot_cmd, _tau_cmd, 
                    _traj_time_vector, 
                    _effort_lims;

    Eigen::MatrixXd _q_p_ref, _q_p_dot_ref, _tau_ref, _dt_opt;

    bool _looped_traj = false, 
         _approach_traj_started = false, _approach_traj_finished = false, 
         _traj_started = false, _traj_finished = false, 
         _first_run = true;

    double _delta_effort_lim,
           _replay_dt, _traj_pause_time,
           _loop_time = 0.0, _plugin_dt,
           _t_exec_traj;

    int _n_jnts_model, 
        _sample_index = 0, _n_samples;

    XBot::ModelInterface::Ptr _model;

    MatLogger2::Ptr _logger;

   // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    void get_params_from_config();
    void init_model_interface();
    bool load_opt_data();
    void send_trajectory();
    void send_approach_traj();
    void saturate_input();
                 
};

#endif // MAT_REPLAYER_RT