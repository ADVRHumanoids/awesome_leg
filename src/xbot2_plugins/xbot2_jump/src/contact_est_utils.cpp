#include "contact_est_utils.hpp"

using namespace ContactEstUtils;

ContactEstimation::ContactEstimation(std::string link_name,
                                     std::vector<int> dofs,
                                     XBot::ModelInterface::Ptr model)
    : _link_name{link_name}, _model{model}, _dofs{dofs}
{

    this->createVirtualFt();

}

void ContactEstimation::createVirtualFt()
{
    using namespace XBot::Cartesian::Utils;

    if(!_ft_estimator)
    {
        _ft_estimator = std::make_shared<ForceEstimation>(
                    _model,
                    ForceEstimation::DEFAULT_SVD_THRESHOLD);
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

    _ft_vs->getForce(_f_estimate); // get the force estimate from
    // the force torque sensor
};
