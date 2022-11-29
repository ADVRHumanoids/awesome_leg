#include "calib_utils.hpp"

using std::tanh;

using namespace CalibUtils;
using namespace SignProcUtils;
using namespace std::chrono;

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
                         double tanh_coeff,
                         double q_dot_3sigma)
  :_K_t{K_t},
   _K_d0{K_d0}, _K_d1{K_d1},
   _rot_MoI{rot_MoI},
   _red_ratio{red_ratio},
   _tanh_coeff{tanh_coeff},
   _q_dot_3sigma{q_dot_3sigma}
{

    _n_jnts = _K_t.size();

    _q_dot = Eigen::VectorXd::Zero(_n_jnts);
    _q_ddot = Eigen::VectorXd::Zero(_n_jnts);
    _tau = Eigen::VectorXd::Zero(_n_jnts);
    _tau_friction = Eigen::VectorXd::Zero(_n_jnts);

    _iq_est = Eigen::VectorXd::Zero(_n_jnts);

    // initialize the awesome sign function
    _sign_with_memory = SignProcUtils::SignWithMem(_q_dot_3sigma, _tanh_coeff);
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

        double static_friction_effort = _K_d0(i) * _sign_with_memory.sign(_q_dot(i));
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


//************* IqCalib *************//

IqCalib::IqCalib()
{

}

IqCalib::IqCalib(int window_size,
                 Eigen::VectorXd K_t,
                 Eigen::VectorXd rot_MoI,
                 Eigen::VectorXd red_ratio,
                 Eigen::VectorXd ig_Kd0,
                 Eigen::VectorXd ig_Kd1,
                 double tanh_coeff,
                 double q_dot_3sigma,
                 double lambda,
                 bool verbose)
  :_window_size{window_size},
    _K_t{K_t}, _rot_MoI{rot_MoI},
    _red_ratio{red_ratio},
    _tanh_coeff{tanh_coeff},
    _q_dot_3sigma{q_dot_3sigma},
    _verbose{verbose},
    _lambda{lambda},
    _ig_Kd0{ig_Kd0},
    _ig_Kd1{ig_Kd1}
{

  // the linear regression problem (for a single joint) is written as
  // A * Kd = tau_friction_measured
  // where A is obtained as [alpha_d0, alpha_d1]
  // and tau_friction_measured is the "measurement" of the fictitious additional
  // friction torque
  // The (unconstrained) optimization problem to be solved is
  // min_{Kd} ||(A * Kd - tau_friction_measured)||^2
  // which is solved by Kd_opt = A_+ * tau_friction_measured
  // This least squared problem can be easily solved employing the builtin
  // utilities of Eigen library (@ https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)

  int err = 0;
//  int err2 = 0;
  int err3 = 0;

  // collecting errors in input dimensions(if any)
  _n_jnts = K_t.size();
  if(rot_MoI.size() != _n_jnts)
  {
      err = err + 1;
  }
  if(red_ratio.size() != _n_jnts)
  {
      err = err + 1;
  }

//  if(_lb_Kd.size() != _n_opt_vars)
//  {
//      err2 = err2 + 1;
//  }
//  if(_ub_Kd.size() != _n_opt_vars)
//  {
//      err2 = err2 + 1;
//  }

  if(_ig_Kd0.size() != _n_jnts)
  {
      err3 = err3 + 1;
  }
  if(_ig_Kd1.size() != _n_jnts)
  {
      err3 = err3 + 1;
  }

  // throwing err. in case an error occurred
  if (err != 0)
  {
      std::string exception = std::string("IqCalib::IqCalib(): dimension mismatch in one or more of the input data -> \n") +
                              std::string("K_t length: ") + std::to_string(_K_t.size()) + std::string("\n") +
                              std::string("rot_MoI length: ") + std::to_string(_rot_MoI.size()) + std::string("\n") +
                              std::string("red_ratio length: ") + std::to_string(_red_ratio.size()) + std::string("\n") +
                              std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

      throw std::invalid_argument(exception);
  }
//  if (err2 != 0)
//  {
//      std::string exception = std::string("IqCalib::IqCalib(): dimension mismatch in one or more of the input data -> \n") +
//                              std::string("lb_Kd length: ") + std::to_string(_lb_Kd.size()) + std::string("\n") +
//                              std::string("ub_Kd length: ") + std::to_string(_ub_Kd.size()) + std::string("\n") +
//                              std::string("which do not match the required length of: ") + std::to_string(_n_opt_vars);

//      throw std::invalid_argument(exception);
//  }
  if (err3 != 0)
  {
      std::string exception = std::string("IqCalib::IqCalib(): dimension mismatch in one or more of the input data -> \n") +
                              std::string("ig_Kd0 length: ") + std::to_string(_ig_Kd0.size()) + std::string("\n") +
                              std::string("ig_Kd1 length: ") + std::to_string(_ig_Kd1.size()) + std::string("\n") +
                              std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

      throw std::invalid_argument(exception);
  }
  else
  {
    _Alpha = Eigen::MatrixXd::Zero(_window_size * _n_jnts, _n_opt_vars);

    _I_lambda = Eigen::MatrixXd::Identity(_n_opt_vars, _n_opt_vars);
    _b_lambda = Eigen::VectorXd::Zero(_I_lambda.rows());

    _A = Eigen::MatrixXd::Zero(_window_size + _I_lambda.rows(), _n_opt_vars);
    _b = Eigen::VectorXd::Zero(_window_size + _I_lambda.rows());

    _tau_total = Eigen::VectorXd::Zero(_n_jnts);
    _tau_friction = Eigen::VectorXd::Zero(_window_size * _n_jnts);

    _Kd0 = Eigen::VectorXd::Zero(_n_jnts);
    _Kd1 = Eigen::VectorXd::Zero(_n_jnts);

    _ig_Kd = Eigen::VectorXd::Zero(_n_opt_vars);
    _lb_Kd = Eigen::VectorXd::Zero(_n_opt_vars);
    _ub_Kd = Eigen::VectorXd::Zero(_n_opt_vars);

    _alpha_d0 = Eigen::VectorXd::Zero(_window_size * _n_jnts);
    _alpha_d1 = Eigen::VectorXd::Zero(_window_size * _n_jnts);

    _q_dot = Eigen::VectorXd::Zero(_n_jnts);
    _q_ddot = Eigen::VectorXd::Zero(_n_jnts);
    _iq = Eigen::VectorXd::Zero(_n_jnts);
    _tau = Eigen::VectorXd::Zero(_n_jnts);

    _sol_time = Eigen::VectorXd::Zero(_n_jnts);

    // initialize the awesome sign-with-memory function
    _sign_with_memory = SignProcUtils::SignWithMem(_q_dot_3sigma, _tanh_coeff);


  }

}

void IqCalib::shift_data(Eigen::VectorXd& data,
                         bool towards_back)
{
    // shifting data towards the back of the qeue
    // (starting from the penultimate sample) of EACH JOINT
    // (recall data from each joint is attached AFTER the data of the previous one)

    int last_sample_index = _window_size - 1; // index of last sample WITHIN each
    // joint

    for (int jnt = 0; jnt < _n_jnts; jnt++)
    { // shifting data for each joint
        if (towards_back)
        {
            int last_sample_index = _window_size - 1;

            for (int i = last_sample_index - 1; i >= 0; i--)
            {
                data(i + 1 + _window_size * jnt) = data(i + _window_size * jnt);
            }
        }
        else
        {
            for (int i = 1; i <= last_sample_index; i++)
            {
                data(i - 1 + _window_size * jnt) = data(i + _window_size * jnt);
            }
        }
    }

}

void IqCalib::shift_data(Eigen::MatrixXd& data,
                         bool rowwise,
                         bool towards_back)
{

    // shifting data towards the back of the qeue
    // (starting from the penultimate sample)
    // by default, shift rows towards bottom

    int last_sample_index = _window_size - 1;

    for (int jnt = 0; jnt < _n_jnts; jnt++)
    { // shifting data for each joint

        if (towards_back)
        {
            for (int i = last_sample_index - 1; i >= 0; i--)
            {

                if (rowwise)
                {
                    data.block(i + 1 + _window_size * jnt, 0, 1, data.cols()) = data.block(i + _window_size * jnt, 0, 1, data.cols());
                }
                else
                {
                    data.block(0, i + 1 + _window_size * jnt, data.rows(), 1) = data.block(0, i + _window_size * jnt, data.rows(), 1);
                }
            }
        }
        else
        {
            for (int i = 1; i <= last_sample_index; i++)
            {

                if (rowwise)
                {
                    data.block(i - 1 + _window_size * jnt, 0, 1, data.cols()) = data.block(i + _window_size * jnt, 0, 1, data.cols());
                }
                else
                {
                    data.block(0, i - 1 + _window_size * jnt, data.rows(), 1) = data.block(0, i + _window_size * jnt, data.rows(), 1);
                }
            }
        }
    }
}

void IqCalib::solve_iq_cal_QP(int jnt_index)
{
    // jnt_index -> 0-based indexing

    // extracting data of joint jnt_index
    _A.block(0, 0, _window_size, _A.cols()) = _Alpha.block(_window_size * jnt_index, 0, _window_size, _Alpha.cols());
    _A.block(_window_size, 0, _I_lambda.rows(), _I_lambda.cols()) = std::sqrt(_lambda) * _I_lambda; // adding regularization

    _b.segment(0, _window_size) = _tau_friction.segment(_window_size * jnt_index, _window_size);
    _b_lambda = _ig_Kd; // the regularization is done around _ig_Kd
    _b.segment(_window_size, _I_lambda.rows()) = std::sqrt(_lambda) * _b_lambda;

    _sol_start = high_resolution_clock::now(); // profiling solution time

    // solving unconstrained linear regression problem with Eigen builtin method
    Eigen::VectorXd opt_Kd = _A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_b);

    _sol_stop = high_resolution_clock::now(); // profiling solution time

    _sol_time(jnt_index) = duration_cast<milliseconds>(_sol_stop - _sol_start).count();

    _Kd0(jnt_index) = opt_Kd(0);
    _Kd1(jnt_index) = opt_Kd(1);
}

void IqCalib::set_ig(Eigen::VectorXd ig_Kd0,
                     Eigen::VectorXd ig_Kd1)
{
    // check on ig dimensions
    int err = 0;
    if (ig_Kd0.size() != _n_jnts)
    {
        err = err + 1;
    }
    if (ig_Kd1.size() != _n_jnts)
    {
        err = err + 1;
    }
    if (err == 0)
    { // all ok --> set igs

        _ig_Kd0 = ig_Kd0;
        _ig_Kd1 = ig_Kd1;

    }
    else
    {
        std::string exception = std::string("IqCalib::get_current_optimal_Kd(): dimension mismatch in one or more of the input data -> \n") +
                                std::string("ig_Kd0 length: ") + std::to_string(ig_Kd0.size()) + std::string("\n") +
                                std::string("ig_Kd1 length: ") + std::to_string(ig_Kd1.size()) + std::string("\n") +
                                std::string("which do not match the required length of: ") + std::to_string(_n_jnts);

        throw std::invalid_argument(exception);
    }

}

void IqCalib::get_sol_millis(Eigen::VectorXd& millis)
{
    millis = _sol_time;
}

void IqCalib::get_current_optimal_Kd(Eigen::VectorXd& Kd0_opt,
                                     Eigen::VectorXd& Kd1_opt)

{
    // just in case the user provides uninitialized vector
    // or vector of wrong dimension

    Kd0_opt = Eigen::VectorXd::Zero(_n_jnts);
    Kd1_opt = Eigen::VectorXd::Zero(_n_jnts);

    for (int i = 0; i < _n_jnts; i++)
    {
        _ig_Kd << _ig_Kd0(i), _ig_Kd1(i); // using the last set ig for Kd
        solve_iq_cal_QP(i);
    }

    // assign output
    Kd0_opt = _Kd0;
    Kd1_opt = _Kd1;

}

void IqCalib::add_sample(Eigen::VectorXd q_dot,
                         Eigen::VectorXd q_ddot,
                         Eigen::VectorXd iq,
                         Eigen::VectorXd tau)
{

    _q_dot = q_dot;
    _q_ddot = q_ddot;
    _iq = iq;
    _tau = tau;

    // shift data one sample behind
    shift_data(_alpha_d0);
    shift_data(_alpha_d1);
    shift_data(_tau_friction);

    // assign current sample values
    compute_alphad0();
    compute_alphad1();
    assemble_Alpha();
    compute_tau_friction();

}


void IqCalib::compute_alphad0()
{

    for (int i = 0; i < _n_jnts; i++)
    {
        _alpha_d0(i * _window_size) =  (double) _sign_with_memory.sign(_q_dot(i)); // assign last sample
    }
}

void IqCalib::compute_alphad1()
{
    for (int i = 0; i < _n_jnts; i++)
    {
        _alpha_d1(i * _window_size) = _q_dot(i); // assign last sample
    }
}

void IqCalib::assemble_Alpha()
{
    _Alpha.block(0, 0, _Alpha.rows(), 1) = _alpha_d0;
    _Alpha.block(0, 1, _Alpha.rows(), 1) = _alpha_d1;
}

void IqCalib::compute_tau_friction()
{
    for (int i = 0; i < _n_jnts; i++)
    {

        _tau_total(i) = 1.0 / _red_ratio(i) *
              ( - _rot_MoI(i) * _q_ddot(i) / _red_ratio(i) + _K_t(i) * _iq(i)); // total torque acting
        // on the motor rotor estimated using the iq measurement and the estimate on the motor axis acceleration.
        // The difference between this component and the torque measured on the link side gives the cumulative unmodeled
        // effort on the rotor caused by dissipative actions present between the link and the rotor itself. Ideally,
        // this difference should be zero.
        // To model this dissipative effects, we use a simple static friction + dynamic friction model:
        // tau_friction = Kd0 * sign(q_dot) * Kd1 * q_dot

        _tau_friction(i * _window_size) = _tau_total(i) - _tau(i); // assign to last sample
    }
}

void IqCalib::get_current_tau_total(Eigen::VectorXd& tau_total)
{
    // just in case the user provides uninitialized vector
    // or vector of wrong dimension

    tau_total = Eigen::VectorXd::Zero(_n_jnts);

    for (int i = 0; i < _n_jnts; i++)
    {
        tau_total(i) = _tau_total(i);
    }

}

void IqCalib::get_current_tau_friction(Eigen::VectorXd& tau_friction)
{
    // just in case the user provides uninitialized vector
    // or vector of wrong dimension

    tau_friction = Eigen::VectorXd::Zero(_n_jnts);

    for (int i = 0; i < _n_jnts; i++)
    {
        tau_friction(i) = _tau_friction(i * _window_size);
    }

}

void IqCalib::get_current_alpha(Eigen::VectorXd& alpha_d0, Eigen::VectorXd& alpha_d1)
{

    // just in case the user provides uninitialized vector
    // or vector of wrong dimension

    alpha_d0 = Eigen::VectorXd::Zero(_n_jnts);
    alpha_d1 = Eigen::VectorXd::Zero(_n_jnts);

    for (int i = 0; i < _n_jnts; i++)
    {
        alpha_d0(i) = _alpha_d0(i * _window_size);
        alpha_d1(i) = _alpha_d1(i * _window_size);
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

//************* NumInt *************//

NumInt::NumInt()
{

}

NumInt::NumInt(int n_jnts, double dt, double T_horizon)
    :_n_jnts{n_jnts}, _dt{dt}, _T_horizon{T_horizon}
{
    _n_intervals = std::round(_T_horizon / _dt);

    _n_samples = _n_intervals + 1;

    _window_data = Eigen::MatrixXd::Zero(_n_jnts, _n_samples);

}

void NumInt::add_sample(Eigen::VectorXd sample)
{
  int sample_size = sample.size();

  if(sample_size != _n_jnts)
  {
      std::string exception = std::string("NumInt::add_sample(): Trying to add a sample of size ") +
                              std::to_string(sample_size) + std::string(", which is different from ") +
                              std::to_string(_n_jnts) + std::string(", (number of joints) \n");

      throw std::invalid_argument(exception);
  }

  // shifting data to the right (discarting most remote sample, which is the
  // one on the extreme right)
  for (int i = _n_samples - 1; i > 0; i--)
  {
     _window_data.block(0, i, _n_jnts, 1) = _window_data.block(0, i - 1, _n_jnts, 1);

  }

  _window_data.block(0, 0 , _n_jnts, 1) = sample; // assign most recent sample

}

void NumInt::get(Eigen::VectorXd& sample_integral)
{
    sample_integral = Eigen::VectorXd::Zero(_n_jnts);

    for(int i = _n_intervals; i > 0; i--)
    {
        sample_integral = sample_integral +
                ( _window_data.block(0, i, _n_jnts, 1) +
                  _window_data.block(0, i - 1, _n_jnts, 1) ) / 2.0 * _dt;
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

//************* SignWithMem *************//

SignWithMem::SignWithMem()
{

}

SignWithMem::SignWithMem(double signal_3sigma,
                         double tanh_coeff)
    :_signal_3sigma{signal_3sigma}, _tanh_coeff{tanh_coeff}
{

    _tanh_thresh = tanh(_tanh_coeff * signal_3sigma);

}

double SignWithMem::approx_sign(double value)
{
    double approx_sign = tanh(_tanh_coeff * value);

    return approx_sign;
}

void SignWithMem::sign_with_memory()
{

    double sign_approximation = approx_sign(_value);

    if (sign_approximation > _tanh_thresh)
    {
        _sign = 1;
    }
    if (sign_approximation <= _tanh_thresh && sign_approximation >= -_tanh_thresh)
    { // uncertainty region --> retain previous value

        _sign = _previous_sign;
    }
    if (sign_approximation < -_tanh_thresh)
    {
        _sign = -1;
    }

    _previous_sign = _sign;
}

int SignWithMem::sign(double value)
{
    _value = value; // assign value

    sign_with_memory(); // compute sign

    return _sign;

}
