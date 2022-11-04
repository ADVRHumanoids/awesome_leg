#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>

#include <XBotInterface/Utils.h>

#include <string>

namespace Eigen
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 3, 1> Vector3d;
}

namespace ContactEstUtils
{

  class ContactEstimation
  {
      public:

          typedef std::weak_ptr<ContactEstimation> WeakPtr;
          typedef std::shared_ptr<ContactEstimation> Ptr;
          typedef std::unique_ptr<ContactEstimation> UniquePtr;

          static UniquePtr MakeEstimator(std::string contact_linkname,
                                         std::vector<int> dofs,
                                         XBot::ModelInterface::Ptr _model); // static method
          // so we can call it without an object of the class

          Eigen::Vector3d get_f();

          Eigen::Vector3d get_w();

          void update_estimate();

      private:

          ContactEstimation(std::string link_name,
                            std::vector<int> dofs,
                            XBot::ModelInterface::Ptr model); // private constructor;
          // can only be called via the factory method

          Eigen::Vector3d _f_estimate_loc, _w_estimate_loc,
                          _f_estimate, _w_estimate;

          XBot::Cartesian::Utils::ForceEstimation::Ptr _ft_estimator;

          XBot::ForceTorqueSensor::ConstPtr _ft_vs;

          std::string _link_name;

          Eigen::Affine3d _contact_link_pose;

          std::vector<int> _dofs;

          XBot::ModelInterface::Ptr _model; // shared instance of
          // the model interface

          void estimate_f();

          void createVirtualFt();

  };

}


// This is the static factory method used to construct instances of the ContactEstimation class
ContactEstUtils::ContactEstimation::UniquePtr ContactEstUtils::ContactEstimation::MakeEstimator(std::string contact_linkname,
                                                                                                std::vector<int> dofs,
                                                                                                XBot::ModelInterface::Ptr model)
{
    return UniquePtr(new ContactEstimation(contact_linkname, dofs, model));
}

