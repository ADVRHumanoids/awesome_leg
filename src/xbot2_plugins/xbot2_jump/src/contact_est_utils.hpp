#include "/home/andreap/hhcm_ws/install/include/base_estimation/base_estimation.h"
#include "/home/andreap/hhcm_ws/install/include/base_estimation/contact_estimation.h"
#include "/home/andreap/hhcm_ws/install/include/base_estimation/vertex_force_optimizer.h"

//#include <base_estimation/base_estimation.h>
//#include <base_estimation.h>

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

          ContactEstimation(std::string link_name); // default constructor

          void get_f(Eigen::Vector3d f_estimate);

          void get_w(Eigen::Vector3d w_estimate);

          void update_estimate();

      private:

          Eigen::Vector3d _f_estimate, _w_estimate;

          ikbe::BaseEstimation::UniquePtr _base_estimation;

          XBot::ForceTorqueSensor::ConstPtr _ft_vs;

          void estimate_f();

          std::string _link_name;
  };

}


