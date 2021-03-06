#ifndef CARTESIO_CNTRL_RT_H
#define CARTESIO_CNTRL_RT_H

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <iostream>

using namespace std;
using namespace XBot;
using namespace XBot::Cartesian;

/**
 * @brief The GravCompCartesio class is a ControlPlugin
 * implementing a s (to be tested on the awesome_leg - pholus).
 */
class GravCompCartesio : public ControlPlugin
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
                    _effort_command,
                    _effort_lims;

    Eigen::Affine3d _target_pose;

    std::string _urdf_path, _srdf_path, _cartesio_path;

    XBot::ModelInterface::Ptr _model;  

    CartesianInterface::Ptr _solver;

    MatLogger2::Ptr _logger;

    double _dt, _time,
           _delta_effort_lim;
    
    int _n_jnts_model;

    // method for computing joint efforts using the measured robot state
    bool get_params_from_config();
    void init_model_interface();
    void init_cartesio_solver();
    void update_state();
    void saturate_input();


};

#endif // GravCompCartesio