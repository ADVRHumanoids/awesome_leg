#include "calib_utils.hpp"

using namespace CalibUtils;
using namespace SignProcUtils;

//************* IqRosGetter *************//

IqRosGetter::IqRosGetter(bool verbose)
  :_verbose{verbose}
{

}

void IqRosGetter::get_last_iq_out(Eigen::VectorXd& iq_out_fb)
{
  iq_out_fb = _iq_out_fb;
}

void IqRosGetter::get_last_iq_out_stamps(Eigen::VectorXd& timestamps)
{
  timestamps = _timestamps;
}

void IqRosGetter::get_time_reference(double& t_ref)
{
  t_ref = _time_ref;
}

void IqRosGetter::set_jnt_names(std::vector<std::string> jnt_names)
{

  _jnt_names = jnt_names;

  _set_jnt_names_from_ros = false;

}

void IqRosGetter::on_js_signal_received(const xbot_msgs::JointState& js_sig)
{

  if(_set_jnt_names_from_ros)
  {
      _jnt_names = js_sig.name;

      _were_jnt_names_set = true; // to signal that we now have read the joint names
  }

  if (_verbose)
  {
      fprintf( stderr, "\n js message received \n");
  }

}

void IqRosGetter::on_aux_signal_received(const xbot_msgs::CustomState& aux_sig)
{

    if (_verbose)
    {
        fprintf( stderr, "\n aux message received \n");
    }

    if (!_were_jnt_names_set)
    {

        auto remapped_aux_tuple = aux_mapper(aux_sig); // remapping aux types

        std::vector<double> ordered_vals = std::get<1>(remapped_aux_tuple); // ordered as _jnt_names

        double* ptr = &ordered_vals[0];

        _iq_out_fb = Eigen::Map<Eigen::VectorXd>(ptr, ordered_vals.size());

    }


}

template <typename T, typename t_v >
int IqRosGetter::find_index(std::vector<T> input_v, t_v value)
{
    /**
    Finds the index of a value in an array.

    @param input_v Input vector.
    @param value Value to be searched within the input vector.

    @return  The index of the element (-1 if not present).
    */

    auto it = find(input_v.begin(), input_v.end(), value);

    // If element was found
    if (it != input_v.end())
    {
        // calculating the index
        int index = it - input_v.begin();
        return index;
    }
    else
    {
        // The element is not present in the vector
        return -1;
    }
}

template <typename T>
std::vector<int> IqRosGetter::map_indices(std::vector<T> input_v1, std::vector<T> input_v2)
{
    /**
    Maps the elements of the first input array to the second.

    @param input_v1 First input vector.
    @param input_v2 Second input vector.

    @return  A vector of indices. indices_map[i] contains the index where the i-th element of input_v1 is present in input_v2.

    */

    int v1_size = input_v1.size(); //initialize output
    std::vector<int> indices_map(v1_size, -1);

    // Here insert a check (size of input_v1 must equal size of input_v2)
    for (int i = 0; i < input_v1.size(); i++)
    {
        indices_map[i] = find_index(input_v2, input_v1[i] );
    }
    return indices_map;
}

int IqRosGetter::get_aux_type_code(std::string msg_type)
{
    /**
    Computes the code ID associated with a given message type and saves this code to the .mat file,
    so that other programs can interpret the signal-type information. To avoid the use of strings, a unique _aux_code suffix is employed.

    @param msg_type The message type name (std::string).
    @return The message code to be associated with the message type name
    */

    int msg_code;

    if (_aux_msg_type_map.find(msg_type) == _aux_msg_type_map.end()) // msg_type not present
    {
        _aux_types_encode_number++; // increment last signal code number by 1
        _aux_msg_type_map.insert({msg_type, _aux_types_encode_number}); // put the code in the dictionary
    }

    msg_code = _aux_msg_type_map.at(msg_type); // read the key associated with the msg_type from the aux code map

    return msg_code;
}

std::tuple<std::vector<int>, std::vector<double>> IqRosGetter::aux_mapper(const xbot_msgs::CustomState& aux_sig)
{

    /**
    Loops through the chains and their joints and, based on the received message, assigns the IDs associated with each message type.

    @param msg Input message
    @return A vector of signal IDs with dimension (number of chains)*(number of joints)
    */

    int n_jnts = aux_sig.name.size(); // number of joints

    std::vector<double> msg_value_remapped(n_jnts, -1.0); // output vector for msg values
    std::vector<int> msg_type_remapped(n_jnts, -1.0); // output vector for msg types

    if (_is_first_aux_sig)
    { // this runs only the first time an aux message is received
      // (joint mapping is assumed to be constant throughout the life of this
      // object)

        _indices = map_indices(_jnt_names, aux_sig.name);

        _is_first_aux_sig = false; // mapping not needed anymore

        _time_ref = aux_sig.header.stamp.toSec(); // this time will be used as a reference
    }

    for (int i = 0; i < n_jnts; i++) // mapping
    {
        if (aux_sig.type[i].find(_iq_sig_basename) != std::string::npos)
        { // we are reading the iq aux type

            int encoded_type = get_aux_type_code(aux_sig.type[i]);
            msg_type_remapped[_indices[i]] = encoded_type;
            msg_value_remapped[_indices[i]] = aux_sig.value[i];

            _timestamps[_indices[i]] = aux_sig.header.stamp.toSec() - _time_ref;// getting timestamp

        }

    }

    return make_tuple(msg_type_remapped, msg_value_remapped);
}

//************* IqEstimator *************//

IqEstimator::IqEstimator(Eigen::VectorXd K_t,
                         Eigen::VectorXd K_d0, Eigen::VectorXd K_d1,
                         Eigen::VectorXd rot_MoI,
                         Eigen::VectorXd red_ratio,
                         double tanh_coeff)
  :_K_t{K_t}, _K_d0{K_d0}, _K_d1{K_d1}, _rot_MoI{rot_MoI}, _red_ratio{red_ratio}, _tanh_coeff{tanh_coeff}
{

  _n_jnts = _K_t.size();

}

IqEstimator::IqEstimator()
{
  _iq_est = Eigen::VectorXd::Zero(_K_t.size());
}

void IqEstimator::get_iq_estimate(std::vector<float>& iq_est)
{
  compute_iq_estimates();

  iq_est.reserve(_n_jnts);

  for (int i = 0; i < _n_jnts; i++)
  {
     iq_est[i] = _iq_est[i]; // mapping Eigen Vector to std::vector
  }

}

void IqEstimator::get_iq_estimate(Eigen::VectorXd& iq_est)
{

  compute_iq_estimates();

  iq_est = _iq_est;

}

void IqEstimator::compute_iq_estimates()
{
  Eigen::VectorXd aux = _tanh_coeff * _q_dot;
  Eigen::VectorXd aux2 = aux.array().tanh();

  Eigen::VectorXd static_friction_effort = _K_d0 * aux2;
  Eigen::VectorXd dynamic_friction_effort = _K_d1 * _q_dot;

  Eigen::VectorXd total_torque_on_motor = _tau + static_friction_effort + dynamic_friction_effort;

  Eigen::VectorXd red_ratio_inv = _red_ratio.inverse();

  Eigen::VectorXd motor_omega_dot = _q_dot * red_ratio_inv;

  Eigen::VectorXd required_motor_torque = _rot_MoI * motor_omega_dot + total_torque_on_motor * _red_ratio;

  Eigen::VectorXd K_t_inv = _K_t.inverse();

  _iq_est = required_motor_torque * K_t_inv;

}

void IqEstimator::set_current_state(Eigen::VectorXd q_dot, Eigen::VectorXd q_ddot, Eigen::VectorXd tau)
{
  _q_dot = q_dot;
  _q_ddot = q_ddot;
  _tau = tau;

}

//************* NumDiff *************//
NumDiff::NumDiff()
{

}

NumDiff::NumDiff(int n_jnts, int order)
  :_n_jnts{n_jnts}, _order{order}
{

  _window_data = Eigen::MatrixXd::Zero(n_jnts, order + 1);

  _Dt = Eigen::VectorXd::Zero(order + 1); // first element always zero
  // (dt only defined between two samples so, for the most remote sample,
  // we set the dt to 0 to indicate that it is not defined on that node)

}

void NumDiff::add_sample(Eigen::VectorXd sample, double dt)
{
  int sample_size = sample.size();

  if(sample_size != _n_jnts)
  {
      std::string exception = std::string("NumDiff::add_sample(): Trying to add a sample of size ") +
                              std::to_string(sample_size) + std::string(", which is different from ") +
                              std::to_string(_n_jnts) + std::string(", (number of joints) \n");

      throw std::invalid_argument(exception);
  }

  // shifting data to the left (discarting most remote sample)
  for (int i = 0; i < _order; i++)
  {
     _window_data.block(0, i , _n_jnts, 1) = _window_data.block(0, i + 1 , _n_jnts, 1);

  }

  // shifting dt values
  for (int i = 0; i < _order - 1; i++)
  {
     _Dt(i + 1) = _Dt(i + 2); // (first element of Dt always zero)
  }

  _window_data.block(0, _order , _n_jnts, 1) = sample; // assign most recent sample

  _Dt[_order] = dt;

}

void NumDiff::dot(Eigen::VectorXd& sample_dot)
{

  if(_order != 1)
  {
      std::string exception = std::string("NumDiff::dot(): order > 1 not supported yet!!\n");

      throw std::invalid_argument(exception);
  }

  Eigen::VectorXd current_sample = _window_data.col(_order);
  Eigen::VectorXd previous_sample = _window_data.col(_order - 1);

  sample_dot = (current_sample - previous_sample) / _Dt[_order]; // super simple num.
  // diff
}
