#ifndef CARTESIO_CNTRL_RT_H
#define CARTESIO_CNTRL_RT_H

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <iostream>

using namespace std;
using namespace XBot;
using namespace XBot::Cartesian;

/**
 * @brief The GravCompRt class is a ControlPlugin
 * implementing a simple gravity compensation (to be tested on the awesome_leg - pholus).
 */
class CartesioCntrlRt : public ControlPlugin
{

public:

    // CartesioCntrlRt();
    
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

private:
    Eigen::VectorXd _PI, _tau_tilde, 
                    _stiffness, _damping, 
                    _q_p, _q_p_dot, _q_p_ddot,
                    _effort_command;
    double _dt;
    std::string _urdf_path, _srdf_path, _cartesio_yaml_path;
    XBot::ModelInterface::Ptr _model;  
    int _n_jnts_model, _n_jnts_robot;

    // method for computing joint efforts using the measured robot state
    void compute_joint_efforts();

};

#endif // CARTESIO_CNTRL_RT_H