#include "contact_est_utils.hpp"

ContactEstUtils::ContactEstimation::ContactEstimation(std::string link_name)
: _link_name{link_name}
{
    _base_estimation
}

void ContactEstUtils::ContactEstimation::get_f(Eigen::Vector3d f_estimate)
{

};

void ContactEstUtils::ContactEstimation::get_w(Eigen::Vector3d w_estimate)
{

};

void ContactEstUtils::ContactEstimation::update_estimate(){};

void ContactEstUtils::ContactEstimation::estimate_f(){};
