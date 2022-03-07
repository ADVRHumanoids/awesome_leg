#ifndef CARTESIO_IMP_CNTRL_ROS_RT
#define CARTESIO_IMP_CNTRL_ROS_RT

#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

using namespace XBot;
using namespace XBot::Cartesian;

/**
 * @brief The CartesioImpCntrlRosRt class is a ControlPlugin
 * implementing a cartesian impedance control, where the target pose
 * is provided by an RViz interactive marker. To send pose references, 
 * run the plugin, launch the marker_spawner node and load the model and
 * the marker in RViz (alternatively, directly launch cartesio_imp_cntrl_marker.launch )
 */
class CartesioImpCntrlRosRt : public ControlPlugin
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

    // callback for 'Stopping' state
    void stopping() override;

    // callback for 'On Stop' state
    void on_stop() override;

    // callback for 'On Abort' state
    void on_close() override;

    // callback for 'On Abort' state
    void on_abort() override;

private:
    Eigen::VectorXd _stiffness, _damping, 
                    _stop_stiffness, _stop_damping,
                    _q_p_meas, _q_p_dot_meas, _q_p_ddot_meas,
                    _effort_command, _meas_effort,
                    _effort_lims;

    Eigen::Affine3d _target_pose;
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

    Eigen::MatrixXd _M;
    Eigen::MatrixXd _J;

    XBot::Cartesian::RosServerClass::Ptr _ros_srv;

    std::unique_ptr<thread> _nrt_thread;
    
    std::unique_ptr<ros::NodeHandle> _nh;
    
    bool _rt_active, _nrt_exit;
    
    double _dt, _time,
           _delta_effort_lim;
    
    int _n_jnts_model, _n_jnts_robot;

    // method for computing joint efforts using the measured robot state
    bool get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void create_ros_api();
    void spawn_rnt_thread();
    void nrt_thread_callback();
    void saturate_input();

};

#endif // CARTESIO_IMP_CNTRL_ROS_RT