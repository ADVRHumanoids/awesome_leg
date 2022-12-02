#include "contact_est.hpp"

using namespace ContactEstUtils;

ContactEstimation::ContactEstimation(std::string link_name,
                                     std::vector<int> dofs,
                                     XBot::ModelInterface::Ptr model,
                                     double plugin_rate)
    : _link_name{link_name}, _model{model}, _dofs{dofs}, _plugin_rate{plugin_rate}
{

    this->createVirtualFt();

}

void ContactEstimation::createVirtualFt()
{
    using namespace XBot::Cartesian::Utils;

    if(!_ft_estimator)
    {
        _ft_estimator = std::make_shared<ForceEstimationMomentumBased>(
                    _model,
                    _plugin_rate,
                    ForceEstimation::DEFAULT_SVD_THRESHOLD,
                    DEFAULT_OBS_BW);
    }

    // generate virtual ft
    _ft_vs = _ft_estimator->add_link(_link_name, _dofs);

}

Eigen::Vector3d ContactEstimation::get_f()
{
    estimate_f();

    Eigen::Vector3d f_estimate = _f_estimate;

    return f_estimate;
};

Eigen::Vector3d ContactEstimation::get_w()
{
    Eigen::Vector3d w_estimate = _w_estimate;

    return w_estimate;
};

void ContactEstimation::update_estimate(){

    _ft_estimator->update(); // updates the force estimator
    // with the current state and effort measurements

};

void ContactEstimation::estimate_f(){

    _ft_vs->getForce(_f_estimate_loc); // get the force estimate from
    // the force torque sensor

    _model->getPose(_link_name, _contact_link_pose);

    _f_estimate = _contact_link_pose * _f_estimate_loc; // rotate force from local
    // to base link frame
};
