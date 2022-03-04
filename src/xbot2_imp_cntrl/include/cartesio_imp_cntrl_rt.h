#ifndef CARTESIO_IMP_CNTRL_RT
#define CARTESIO_IMP_CNTRL_RT

#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>

#include <iostream>
#include <cartesian_interface/ros/RosClient.h>

using namespace std;
using namespace XBot;
using namespace XBot::Cartesian;

/**
 * @brief The CartesioImpCntrlRt class is a real-time ControlPlugin
 * implementing a point-to-point motion, with cartesian impedance control.
 * The target joint positions are specified in xbot2_imp_cntrl_rt.yaml,
 * in the tag associated to this plugin.
 */
class CartesioImpCntrlRt : public ControlPlugin
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
    void on_stop() override;

private:

    Eigen::VectorXd _tau_tilde, 
                    _stiffness, _damping, 
                    _stop_stiffness, _stop_damping,
                    _q_p_meas, _q_p_dot_meas, _q_p_ddot_meas,
                    _q_p_ci, _q_p_dot_ci, _q_p_ddot_ci,
                    _q_p_target,
                    _effort_command, _meas_effort,
                    _effort_lims;

    Eigen::Affine3d _target_pose;

    std::string _urdf_path, _srdf_path, _cartesio_path;

    XBot::ModelInterface::Ptr _model;  

    CartesianInterface::Ptr _solver;

    InteractionTask::Ptr _int_task;

    MatLogger2::Ptr _logger;

    Impedance _impedance;
    Eigen::Matrix6d _cart_stiffness;
    Eigen::Matrix6d _cart_damping;

    double _dt, _time, _t_exec,
           _delta_effort_lim;;

    int _n_jnts_model, _n_jnts_robot;

    // method for computing joint efforts using the measured robot state
    bool get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void saturate_input();

};

#endif // CARTESIO_IMP_CNTRL_RT