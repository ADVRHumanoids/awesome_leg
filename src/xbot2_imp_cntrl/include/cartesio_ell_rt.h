#ifndef CARTESIO_ELL_RT
#define CARTESIO_ELL_RT

#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <xbot2/ros/ros_support.h>

#include <math.h> 

#include <awesome_leg_pholus/EllTrajRt.h>

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
class CartesioEllRt : public ControlPlugin
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
    
    Eigen::VectorXd _tau_tilde, 
                    _stiffness, _damping, 
                    _stop_stiffness, _stop_damping,
                    _q_p_meas, _q_p_dot_meas,
                    _effort_command, _meas_effort, 
                    _tip_ref_traj,
                    _effort_lims;

    Eigen::Affine3d _target_pose;
    Eigen::Vector6d _target_vel, _target_acc;

    Eigen::Affine3d _meas_pose;

    std::string _urdf_path, _srdf_path, _cartesio_path;

    XBot::ModelInterface::Ptr _model, _nrt_model; 
     
    CartesianInterfaceImpl::Ptr _solver;
    LockfreeBufferImpl::Ptr _nrt_solver;

    CartesianTask::Ptr _cart_task;
    InteractionTask::Ptr _int_task;

    MatLogger2::Ptr _logger;

    Impedance _impedance;
    Eigen::Matrix6d _cart_stiffness;
    Eigen::Matrix6d _cart_damping;

    XBot::Cartesian::RosServerClass::Ptr _ros_srv;

    std::unique_ptr<thread> _nrt_thread;

    std::unique_ptr<ros::NodeHandle> _nh;

    // queue object to handle multiple subscribers/servers at once
    CallbackQueue _queue;

    // handle adapting ROS primitives for RT support
    RosSupport::UniquePtr _ros;

    bool _rt_active, _nrt_exit, _is_interaction,
         _traj_par_callback_trigger = false,
         _use_vel_ff, _use_acc_ff,
         _is_forward;

    double _dt, _time, 
           _time_traj_par,
           _traj_prm_rmp_time, _t_exec_lb,
                               _t_exec_traj, _a_ellps, _b_ellps, _x_c_ellps, _z_c_ellps, _alpha,
                               _t_exec_traj_init, _a_ellps_init, _b_ellps_init, _x_c_ellps_init, _z_c_ellps_init, _alpha_init,
                               _t_exec_traj_trgt, _a_ellps_trgt, _b_ellps_trgt, _x_c_ellps_trgt, _z_c_ellps_trgt, _alpha_trgt,
           _delta_effort_lim;

    int _n_jnts_model, _n_jnts_robot;

    bool get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void create_ros_api();
    void spawn_rnt_thread();
    void nrt_thread_callback();
    void compute_ref_traj(double time);
    void saturate_input();
    void peisekah_transition();

    // ROS topic callback
    void on_ell_traj_recv(const awesome_leg_pholus::EllTrajRt& msg);

    // XBot2.0 pub/sub/server wrapping the ones from ROS
    SubscriberPtr<awesome_leg_pholus::EllTrajRt> _ell_traj_sub;

};

#endif // CARTESIO_ELL_RT