#ifndef BYPASS_DSP_RT
#define BYPASS_DSP_RT

#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <xbot2/ros/ros_support.h>

#include <math.h> 

#include <awesome_leg_pholus/SinJointTraj.h>
#include <awesome_leg_pholus/BypassDspRt.h>

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
class BypassDsp : public ControlPlugin
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
    
    Eigen::VectorXd
                    _stiffness, _damping, 
                    _stop_stiffness, _stop_damping,
                    _q_p_meas, _q_p_dot_meas,
                    _q_p_trgt, _q_p_dot_trgt,
                    _q_min, _q_max,
                    _q_dot_max,  
                    _effort_command, _meas_effort, 
                    _effort_lims,
                    _jnt_imp_lims;

    std::string _urdf_path, _srdf_path, _cartesio_path;

    XBot::ModelInterface::Ptr _model, _nrt_model; 
     
    MatLogger2::Ptr _logger;

    std::unique_ptr<thread> _nrt_thread;

    std::unique_ptr<ros::NodeHandle> _nh;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    bool _rt_active, _nrt_exit,
         _traj_par_callback_trigger = false, _imp_callback_trigger = false,
         _first_run = true, 
         _use_motor_side_readings,
         _bypass_dsp;

    double _time_traj_par, _time_jnt_imp,
           _dt, 
           _traj_prm_rmp_time, _imp_rmp_time,
           _t_exec_lb,        
           _delta_effort_lim;

    Eigen::VectorXd _time, _t_exec, _center, _phase_off, _overshoot,
                    _t_exec_init, _center_init, _phase_off_init, _overshoot_init, 
                    _stiffness_init, _damping_init,
                    _t_exec_trgt, _center_trgt, _phase_off_trgt, _overshoot_trgt,
                    _stiffness_trgt, _damping_trgt;

    int _n_jnts_robot;

    bool get_params_from_config();
    void update_state();
    void compute_ref_traj(Eigen::VectorXd time);
    void saturate_input();
    void peisekah_transition();
    void compute_jnt_imp_cntrl();

    // ROS service callbacks
    bool on_sin_traj_recv_srv(const awesome_leg_pholus::SinJointTrajRequest& req,
                          awesome_leg_pholus::SinJointTrajResponse& res);

    bool on_jnt_imp_setpoint_recv_srv(const awesome_leg_pholus::BypassDspRtRequest& req,
                          awesome_leg_pholus::BypassDspRtResponse& res);

    // XBot2.0 servers wrapping the ones from ROS
    ServiceServerPtr<awesome_leg_pholus::SinJointTrajRequest,
                     awesome_leg_pholus::SinJointTrajResponse> _sin_traj_srv;

    ServiceServerPtr<awesome_leg_pholus::BypassDspRtRequest,
                     awesome_leg_pholus::BypassDspRtResponse> _bypass_dsp_srv;
                     
};

#endif 