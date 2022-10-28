//#include "/home/andreap/hhcm_ws/install/include/base_estimation/base_estimation.h"
//#include "/home/andreap/hhcm_ws/install/include/base_estimation/contact_estimation.h"
//#include "/home/andreap/hhcm_ws/install/include/base_estimation/vertex_force_optimizer.h"

#include <base_estimation/base_estimation.h>
#include <base_estimation/contact_estimation.h>
#include <base_estimation/vertex_force_optimizer.h>

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

          static UniquePtr MakeEstimator(std::string contact_linkname); // static method
          // so we can call it without an object of the class

          void get_f(Eigen::Vector3d f_estimate);

          void get_w(Eigen::Vector3d w_estimate);

          void update_estimate();

      private:

          ContactEstimation(std::string link_name); // private constructor;
          // can only be called via the factory method

          Eigen::Vector3d _f_estimate, _w_estimate;

          ikbe::BaseEstimation::UniquePtr _base_estimation;

          XBot::ForceTorqueSensor::ConstPtr _ft_vs;

          void estimate_f();

          std::string _link_name;
  };

}


// This is the static factory method used to construct instances of the ContactEstimation class
ContactEstUtils::ContactEstimation::UniquePtr ContactEstUtils::ContactEstimation::MakeEstimator(std::string contact_linkname)
{
    return UniquePtr(new ContactEstimation(contact_linkname));
}

