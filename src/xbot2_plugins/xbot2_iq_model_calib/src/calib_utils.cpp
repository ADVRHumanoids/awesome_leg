#include "calib_utils.hpp"

using std::tanh;

using namespace CalibUtils;
using namespace SignProcUtils;

//************* IqRosGetter *************//

IqRosGetter::IqRosGetter(bool verbose)
  :_verbose{verbose}
{

}

void IqRosGetter::init_vars()
{
    _n_active_jnts = _jnt_names.size();

    _iq_out_fb = Eigen::VectorXd::Zero(_n_active_jnts);
    _timestamps = Eigen::VectorXd::Zero(_n_active_jnts);
}

void IqRosGetter::get_last_iq_out(Eigen::VectorXd& iq_out_fb)
{
    if(_vars_were_initialized)
    { // only assign output if _iq_out_fb has a nonzero dimension
      // (guaranteed after init_vars is called)
        iq_out_fb = _iq_out_fb;
    }
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

    _were_jnt_names_set = true;

}

void IqRosGetter::get_jnt_names(std::vector<std::string>& jnt_names)
{

    jnt_names = _jnt_names;

}

bool IqRosGetter::is_iq_out_topic_active()
{

    if(!_is_first_aux_sig)
    { // we have received at least one aux signal --> which means that at this point we know for sure
      // the joint names associated with each message

        return true;
    }
    else
    {
        return false;
    }

}

void IqRosGetter::on_js_signal_received(const xbot_msgs::JointState& js_sig)
{

    if(_set_jnt_names_from_ros && !_were_jnt_names_set)
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

    if(_is_first_aux_sig)
    {
        init_vars();

        _vars_were_initialized = true;
    }

    auto remapped_aux_tuple = aux_mapper(aux_sig); // remapping aux types

    std::vector<double> ordered_vals = std::get<1>(remapped_aux_tuple); // ordered as _jnt_names
    double* ptr = &ordered_vals[0];

    _iq_out_fb = Eigen::Map<Eigen::VectorXd>(ptr, ordered_vals.size());

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

    _q_dot = Eigen::VectorXd::Zero(_n_jnts);
    _q_ddot = Eigen::VectorXd::Zero(_n_jnts);
    _tau = Eigen::VectorXd::Zero(_n_jnts);
    _tau_friction = Eigen::VectorXd::Zero(_n_jnts);

    _iq_est = Eigen::VectorXd::Zero(_n_jnts);
}

IqEstimator::IqEstimator()
{

}

void IqEstimator::get_iq_estimate(std::vector<float>& iq_est)
{
    int err = 0;
    if(iq_est.size() != _n_jnts)
    {
        err = err + 1;
    }

    if (err != 0)
    {
        std::string exception = std::string("IqEstimator::get_iq_estimate(): dimension mismatch of input data \n");
    }

    compute_iq_estimates();

    iq_est = std::vector<float>(_n_jnts);

    for (int i = 0; i < _n_jnts; i++)
    {
     iq_est[i] = _iq_est[i]; // mapping Eigen Vector to std::vector(brutal way)
    }

}

void IqEstimator::get_iq_estimate(Eigen::VectorXd& iq_est)
{

    int err = 0;
    if(iq_est.size() != _n_jnts)
    {
        err = err + 1;
    }

    if (err != 0)
    {
        std::string exception = std::string("IqEstimator::get_iq_estimate(): dimension mismatch of input data \n");
    }

    compute_iq_estimates();

    iq_est = _iq_est;

}

void IqEstimator::get_iq_estimate(std::vector<float>& iq_est,
                                  Eigen::VectorXd K_d0, Eigen::VectorXd K_d1)
{
    int err = 0;
    if(K_d0.size() != _n_jnts)
    {
        err = err + 1;
    }
    if(K_d1.size() != _n_jnts)
    {
        err = err + 1;
    }

    if (err != 0)
    {
        std::string exception = std::string("IqEstimator::compute_iq_estimates(): dimension mismatch of input data -> \n") +
                                std::string("K_d0 length: ") + std::to_string(K_d0.size()) + std::string("\n") +
                                std::string("K_d1 length: ") + std::to_string(K_d1.size()) + std::string("\n") +
                                std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

        throw std::invalid_argument(exception);
    }
    else
    {
        // update K_d0 and K_d1 with the provided values
        _K_d0 = K_d0;
        _K_d1 = K_d1;

        compute_iq_estimates(); // compute iq estimate with the runtime set gains

        iq_est = std::vector<float>(_n_jnts);

        for (int i = 0; i < _n_jnts; i++)
        {
           iq_est[i] = _iq_est[i]; // mapping Eigen Vector to std::vector (brutal way)
        }

    }

}

void IqEstimator::get_iq_estimate(Eigen::VectorXd& iq_est,
                                  Eigen::VectorXd K_d0, Eigen::VectorXd K_d1)
{
    int err = 0;
    if(K_d0.size() != _n_jnts)
    {
        err = err + 1;
    }
    if(K_d1.size() != _n_jnts)
    {
        err = err + 1;
    }

    if (err != 0)
    {
        std::string exception = std::string("IqEstimator::compute_iq_estimates(): dimension mismatch of input data -> \n") +
                                std::string("K_d0 length: ") + std::to_string(K_d0.size()) + std::string("\n") +
                                std::string("K_d1 length: ") + std::to_string(K_d1.size()) + std::string("\n") +
                                std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

        throw std::invalid_argument(exception);
    }
    else
    {
        // update K_d0 and K_d1 with the provided values
        _K_d0 = K_d0;
        _K_d1 = K_d1;

        compute_iq_estimates(); // compute iq estimate with the runtime set gains

        iq_est = _iq_est;

    }

}

void IqEstimator::get_tau_link(Eigen::VectorXd& tau)
{
    tau = _tau;
}

void IqEstimator::get_tau_friction(Eigen::VectorXd& tau_friction)
{
    tau_friction = _tau_friction;
}

void IqEstimator::get_q_ddot(Eigen::VectorXd& q_ddot)
{
    q_ddot = _q_ddot;
}

void IqEstimator::compute_iq_estimates()
{

    for (int i = 0; i < _n_jnts; i++)
    {

        double static_friction_effort = _K_d0(i) * tanh(_tanh_coeff * _q_dot(i));
        double dynamic_friction_effort = _K_d1(i) * _q_dot(i);

        _tau_friction(i) = static_friction_effort + dynamic_friction_effort;

        double total_torque_on_motor = _tau(i) + _tau_friction(i);

        double motor_omega_dot = _q_ddot(i) / _red_ratio(i);

        double required_motor_torque = _rot_MoI(i) * motor_omega_dot + total_torque_on_motor * _red_ratio(i);

        _iq_est(i) = required_motor_torque / _K_t(i);
    }

}

void IqEstimator::set_current_state(Eigen::VectorXd q_dot, Eigen::VectorXd q_ddot, Eigen::VectorXd tau)
{
  int err = 0;
  if(q_dot.size() != _n_jnts)
  {
      err = err + 1;
  }
  if(q_ddot.size() != _n_jnts)
  {
      err = err + 1;
  }
  if(tau.size() != _n_jnts)
  {
      err = err + 1;
  }

  if (err != 0)
  {
      std::string exception = std::string("IqEstimator::set_current_state(): dimension mismatch of input data -> \n") +
                              std::string("q_dot length: ") + std::to_string(q_dot.size()) + std::string("\n") +
                              std::string("q_ddot length: ") + std::to_string(q_ddot.size()) + std::string("\n") +
                              std::string("tau length: ") + std::to_string(tau.size()) + std::string("\n") +
                              std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

      throw std::invalid_argument(exception);
  }
  else
  {
      _q_dot = q_dot;
      _q_ddot = q_ddot;
      _tau = tau;

  }

}

//************* NumDiff *************//

NumDiff::NumDiff()
{

}

NumDiff::NumDiff(int n_jnts, double dt, int order)
    :_n_jnts{n_jnts}, _order{order}, _dt{dt}
{

    _spline_interp = std::vector<interp::spline>(_n_jnts);
    _aux_time_vect = Eigen::VectorXd::Zero(_order + 1);
    for (int i = 1; i < _aux_time_vect.size(); i++)
    {
        _aux_time_vect(i) = _aux_time_vect(i - 1) + _dt;
    }
    _k_n1 = Eigen::VectorXd::Zero(2); // 1st order
    _k_n2 = Eigen::VectorXd::Zero(3); // 2nd order
    _k_n3 = Eigen::VectorXd::Zero(4); // 3rd order
    _k_n4 = Eigen::VectorXd::Zero(5); // 4th order
    _k_n5 = Eigen::VectorXd::Zero(6); // 5th order
    _k_n6 = Eigen::VectorXd::Zero(7); // 6th order

    _k_n1 << 1.0,         -1.0; // 1st order
    _k_n2 << 3.0/2.0,     -2.0,   1.0/2.0; // 2nd order
    _k_n3 << 11.0/6.0,    -3.0,   3.0/2.0,   -1.0/3.0; //3rd order
    _k_n4 << 25.0/12.0,   -4.0,       3.0,   -4.0/3.0,    1.0/4.0; //4th order
    _k_n5 << 137.0/60.0,  -5.0,       5.0,  -10.0/3.0,  15.0/12.0,  -1.0/5.0; //5th order
    _k_n6 << 147.0/60.0,  -6.0,  15.0/2.0,  -20.0/3.0,   15.0/4.0,  -6.0/5.0,  1.0/6.0; //6th order

    _window_data = Eigen::MatrixXd::Zero(_n_jnts, _order + 1);

    if(_order == 1)
    {
        _k_n = _k_n1;
    }

    if(_order == 2)
    {
        _k_n = _k_n2;
    }

    if(_order == 3)
    {
        _k_n = _k_n3;
    }

    if(_order == 4)
    {
        _k_n = _k_n4;
    }

    if(_order == 5)
    {
        _k_n = _k_n5;
    }

    if(_order == 6)
    {
        _k_n = _k_n6;
    }

}

void NumDiff::add_sample(Eigen::VectorXd sample)
{
  int sample_size = sample.size();

  if(sample_size != _n_jnts)
  {
      std::string exception = std::string("NumDiff::add_sample(): Trying to add a sample of size ") +
                              std::to_string(sample_size) + std::string(", which is different from ") +
                              std::to_string(_n_jnts) + std::string(", (number of joints) \n");

      throw std::invalid_argument(exception);
  }

  // shifting data to the right (discarting most remote sample, which is the
  // one on the extreme right)
  for (int i = _order - 1; i >= 0; i--)
  {
     _window_data.block(0, i + 1 , _n_jnts, 1) = _window_data.block(0, i, _n_jnts, 1);

  }

  _window_data.block(0, 0 , _n_jnts, 1) = sample; // assign most recent sample

}

void NumDiff::dot(Eigen::VectorXd& sample_dot, bool use_spline)
{

  if(_order > 6)
  {
      // forcing spline-based derivative estimation !!

      use_spline = true;
  }

  if (!use_spline)
  {
    sample_dot = (_window_data * _k_n) / _dt; // estimating derivative with backward method
    // of order _order
  }
  else
  {

      std::vector<double> x(_aux_time_vect.size());
      std::vector<double> y(_window_data.cols());

      Eigen::Map<Eigen::VectorXd>(&x[0], _aux_time_vect.size(), 1) = _aux_time_vect; // mapping to time vector (same
      // for all dimensions)

      for (int i = 0; i < _n_jnts; i++)
      { // looping through dimensions

          Eigen::Map<Eigen::VectorXd>(&y[0], _window_data.cols(), 1) = _window_data.row(i); // mapping to data vector y
          _spline_interp[i] = interp::spline(x, y); // building spline

          // ADD SPLINE CONTINUITY CONSTRAINTS !!!!!!!!!!!//
          sample_dot(i) = _spline_interp[i].deriv(1, _aux_time_vect(_aux_time_vect.size() - 1)); // evaluating spline first derivative at
          // the current sample
      }
  }
}

//************* MovAvrgFilt *************//

MovAvrgFilt::MovAvrgFilt()
{

}

MovAvrgFilt::MovAvrgFilt(int n_jnts, double dt, int window_size)
    :_n_jnts{n_jnts}, _window_size{window_size}, _samples_dt{dt}
{

    if (_window_size <= 1)
    {
        _window_size = 2;
    }

    _cutoff_freq = _magic_number/_samples_dt * 1.0 / std::sqrt(std::pow(_window_size, 2) - 1);

    _window_data = Eigen::MatrixXd::Zero(_n_jnts, _window_size);

}

MovAvrgFilt::MovAvrgFilt(int n_jnts, double dt, double cutoff_freq)
    :_n_jnts{n_jnts}, _cutoff_freq{cutoff_freq}, _samples_dt{dt}
{

    double nominal_window_size = std::sqrt(std::pow(_magic_number / (_samples_dt * _cutoff_freq), 2) + 1);
    _window_size = std::round(nominal_window_size);
    if (_window_size <= 1)
    {
        _window_size = 2;
    }
    _window_data = Eigen::MatrixXd::Zero(_n_jnts, _window_size);

}

void MovAvrgFilt::add_sample(Eigen::VectorXd sample)
{

  int sample_size = sample.size();

  if(sample_size != _n_jnts)
  {
      std::string exception = std::string("MovAvrgFilt::add_sample(): Trying to add a sample of size ") +
                              std::to_string(sample_size) + std::string(", which is different from ") +
                              std::to_string(_n_jnts) + std::string(", (number of joints) \n");

      throw std::invalid_argument(exception);
  }

  // shifting data to the right (discarting most remote sample, which is the
  // one on the extreme right)
  for (int i = _window_size - 2; i >= 0; i--)
  {
     _window_data.block(0, i + 1 , _n_jnts, 1) = _window_data.block(0, i, _n_jnts, 1);

  }

  _window_data.block(0, 0 , _n_jnts, 1) = sample; // assign most recent sample

  if (_is_first_run)
  { // if it's the first time we add a sample
    // we fill the empty part of the window with
    // replicae of the added sample.
    // after _window_size samples, the filter is at
    // regime.

    for (int i = 1; i < _window_data.cols(); i++)
    {
        _window_data.block(0, i, _n_jnts, 1) = sample;
    }

    _is_first_run = false;

  }


}

void MovAvrgFilt::get(Eigen::VectorXd& filt_sample)
{

    filt_sample = 1.0 / _window_size * _window_data.rowwise().sum();

}

void MovAvrgFilt::get_cutoff_freq(double& cutoff_f)
{
    cutoff_f = _cutoff_freq;
}

void MovAvrgFilt::get_window_size(int& window_size)
{
    window_size = _window_size;
}

//************* IqCalib *************//

IqCalib::IqCalib()
{

}

IqCalib::IqCalib(int window_size,
                 Eigen::VectorXd K_t,
                 Eigen::VectorXd rot_MoI,
                 Eigen::VectorXd red_ratio,
                 double tanh_coeff)
  :_window_size{window_size},
    _K_t{K_t}, _rot_MoI{rot_MoI},
    _red_ratio{red_ratio},
    _tanh_coeff{tanh_coeff}
{

  // the linear regression problem (for a single joint) is written as
  // A * Kd = tau_fricton_measured
  // where A is obtained as [alpha_d0, alpha_d1]
  // and tau_friction_measured is the "measurement" of the fictitious additional
  // friction torque
  // The (unconstrained) optimization problem to be solved is
  // min_{Kd} ||(A * Kd - tau_friction_measured)||^2
  // which is solved by Kd_opt = A_+ * tau_friction_measured
  // This least squared problem can be easily solved employing the builtin
  // utilities of Eigen library (@ https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)

  _n_jnts = K_t.size();

  int err = 0;
  if(rot_MoI.size() != _n_jnts)
  {
      err = err + 1;
  }
  if(red_ratio.size() != _n_jnts)
  {
      err = err + 1;
  }

  if (err != 0)
  {
      std::string exception = std::string("IqCalib::IqCalib(): dimension mismatch in one or more of the input data -> \n") +
                              std::string("K_t length: ") + std::to_string(_K_t.size()) + std::string("\n") +
                              std::string("rot_MoI length: ") + std::to_string(_rot_MoI.size()) + std::string("\n") +
                              std::string("red_ratio length: ") + std::to_string(_red_ratio.size()) + std::string("\n") +
                              std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

      throw std::invalid_argument(exception);
  }
  else
  {
      _A = Eigen::MatrixXd::Zero(_window_size * _n_jnts, 2);
      _tau_friction = Eigen::VectorXd::Zero(_window_size * _n_jnts);

      _Kd0 = Eigen::VectorXd::Zero(_n_jnts);
      _Kd1 = Eigen::VectorXd::Zero(_n_jnts);

      _Kd = Eigen::VectorXd::Zero(2);

      _alpha_d0 = Eigen::VectorXd::Zero(_window_size * _n_jnts);
      _alpha_d1 = Eigen::VectorXd::Zero(_window_size * _n_jnts);

      _K_t = Eigen::VectorXd::Zero(_n_jnts);
      _rot_MoI = Eigen::VectorXd::Zero(_n_jnts);
      _red_ratio = Eigen::VectorXd::Zero(_n_jnts);
      _tanh_coeff = Eigen::VectorXd::Zero(_n_jnts);

      _q_dot = Eigen::VectorXd::Zero(_n_jnts);
      _q_ddot = Eigen::VectorXd::Zero(_n_jnts);
      _iq = Eigen::VectorXd::Zero(_n_jnts);
      _tau = Eigen::VectorXd::Zero(_n_jnts);
  }

}
