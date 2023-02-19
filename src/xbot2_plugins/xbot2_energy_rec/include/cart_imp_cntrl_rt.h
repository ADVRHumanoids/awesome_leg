#ifndef CART_IMP_CNTRL_RT
#define CART_IMP_CNTRL_RT

#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/problem/Cartesian.h>
#include <cartesian_interface/sdk/problem/Postural.h>

#include <cartesio_acceleration_support/TorqueLimits.h>

#include <iostream>
#include <cartesian_interface/ros/RosClient.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <awesome_leg/PostImpImpStatus.h>

using namespace std;
using namespace XBot;

/**
 * @brief
 */
class CartImpCntrlRt : public ControlPlugin
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

    bool _rt_active, _nrt_exit,
        _is_sim = true, _is_dummy = false;

    int _n_jnts_model, _n_jnts_robot;

    std::string _urdf_path, _srdf_path, _ci_yaml_path,
                _mat_path, _dump_mat_suffix,
                _tip_link_name, _base_link_name, _test_rig_linkname,
                _hw_type;

    double _plugin_dt,
           _loop_time = 0.0, _loop_timer_reset_time = 3600.0,
           _matlogger_buffer_size = 1e6,
           _torque_cntrl_stiffness_setpoint = 0.0,
           _torque_cntrl_damping_setpoint = 0.0;

    Eigen::VectorXd _q_meas, _q_dot_meas, _q_cmd, _q_dot_cmd,
                    _q_model, _v_model, _v_dot_model, _tau_model,
                    _jnt_stiffness_setpoint, _jnt_damping_setpoint,
                    _tau_ff,
                    _tau_cmd,
                    _tau_meas,
                    _effort_lims;

    std::vector<std::string> _task_list;

    XBot::ModelInterface::Ptr _model;  

    Cartesian::CartesianInterfaceImpl::Ptr _ci_solver;

    Cartesian::CartesianTask::Ptr _ground_contact;
    Cartesian::InteractionTask::Ptr _cart_impedance_task;
    Cartesian::PosturalTask::Ptr _actuated_jnt_tracking;
    Cartesian::PosturalTask::Ptr _touchdown_conf;
    Cartesian::acceleration::TorqueLimits::Ptr _torque_limits;

    MatLogger2::Ptr _logger;

    Cartesian::Impedance _cart_impedance, _cart_impedance_setpoint;
    Eigen::Vector6d _k_setpoint, _k;
    Eigen::Vector6d _d_setpoint, _d;
    Eigen::Matrix6d _K_setpoint, _K;
    Eigen::Matrix6d _D_setpoint, _D;

    Eigen::Matrix6d _Lambda_inv;
    Eigen::MatrixXd _Jc, _B;

    Eigen::Vector6d _v_ref, _a_ref;
    Eigen::Vector3d _pos_ref;
    Eigen::Matrix3d _R_ref;
    Eigen::Affine3d _M_ref_imp;

    void init_vars();
    void init_logger();
    void add_data2logger();

    void get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void saturate_input();

    void init_clocks();
    void update_clocks();

    void update_tasks();
    void update_ci_solver();

    void compute_inverse_dyn();

    void is_sim(std::string);
    void is_dummy(std::string);

};

#endif // CART_IMP_CNTRL_RT
