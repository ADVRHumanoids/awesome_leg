#ifndef MAT_REPLAYER_RT
#define MAT_REPLAYER_RT

#include "/home/andreap/matlogger_ws/install/include/matlogger2/matlogger2.h"
// #include <matlogger2/matlogger2.h>

#include <xbot2/xbot2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <xbot2/ros/ros_support.h>

#include <math.h> 

// #include <awesome_leg_pholus/EllTrajRt.h>

#include <thread>

#include "plugin_utils.h"

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
    
    Eigen::VectorXd _bias_tau,
                    _stiffness, _damping, 
                    _stop_stiffness, _stop_damping,
                    _q_p_meas, _q_p_dot_meas,
                    _effort_command, _meas_effort, 
                    _effort_lims;

    Eigen::Affine3d _meas_pose;

    std::string _urdf_path, _srdf_path;

    XBot::ModelInterface::Ptr _model, _nrt_model; 

    MatLogger2::Ptr _logger;

    Eigen::MatrixXd _M;

    XBot::Cartesian::RosServerClass::Ptr _ros_srv;

    std::unique_ptr<thread> _nrt_thread;

    std::unique_ptr<ros::NodeHandle> _nh;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    bool _rt_active, _nrt_exit,
         _traj_par_callback_trigger = false,
         _first_run = true;

    double _dt, _time, 
           _time_traj_par,
           _traj_prm_rmp_time, _t_exec_lb,
           _delta_effort_lim;

    int _n_jnts_model, _n_jnts_robot;

    bool get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void create_ros_api();
    void spawn_rnt_thread();
    void nrt_thread_callback();
    void saturate_input();
    void peisekah_transition();

    // ROS service callback
    // bool on_ell_traj_recv_srv(const awesome_leg_pholus::EllTrajRtRequest& req,
    //                       awesome_leg_pholus::EllTrajRtResponse& res);

    // XBot2.0 server wrapping the ones from ROS
    // ServiceServerPtr<awesome_leg_pholus::EllTrajRtRequest,
    //                  awesome_leg_pholus::EllTrajRtResponse> _ell_traj_srv;
                     
};

#endif // MAT_REPLAYER_RT