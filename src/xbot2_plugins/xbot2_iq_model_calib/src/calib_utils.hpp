#ifndef CALIB_UTILS_H
#define CALIB_UTILS_H

#include <math.h> 

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <vector>
#include <map>

#include <xbot2/ros/ros_support.h>

#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>

#include <xbot_msgs/CustomState.h>
#include <xbot_msgs/JointState.h>

#include <map>
#include <vector>
#include "spline.h"

using namespace XBot;

namespace CalibUtils{

    class IqRosGetter
    {

      public:

        IqRosGetter(bool verbose = false);

        void on_aux_signal_received(const xbot_msgs::CustomState& aux_sig);

        void on_js_signal_received(const xbot_msgs::JointState& js_sig);

        void get_last_iq_out(Eigen::VectorXd& iq_out_fb);

        void get_last_iq_out_stamps(Eigen::VectorXd& timestamps);

        void get_time_reference(double& t_ref);

        void set_jnt_names(std::vector<std::string> jnt_names);

        bool is_iq_out_topic_active();

        void get_jnt_names(std::vector<std::string>& jnt_names);

      private:

        bool _is_first_aux_sig = true,
             _vars_were_initialized = false,
             _were_jnt_names_set = false, _set_jnt_names_from_ros = true,
             _verbose = false;

        double _time_ref = 0.0;

        std::string _iq_sig_basename = "iq_out_fb";

        int _aux_types_encode_number = 0; // global counter used to assign incremental values to aux signal types

        int _n_active_jnts = 0;

        std::vector<int> _indices; // for holding the joint mapping

        std::map<std::string, int> _aux_msg_type_map;
        std::vector<std::string> _jnt_names; // auxiliary vector where the joint names (as visible in the js message) are saved.

        Eigen::VectorXd _iq_out_fb, _timestamps;

        void init_vars();

        template <typename T, typename t_v >
        int find_index(std::vector<T> input_v, t_v value);

        template <typename T>
        std::vector<int> map_indices(std::vector<T> input_v1, std::vector<T> input_v2);

        int get_aux_type_code(std::string msg_type);

        std::tuple<std::vector<int>, std::vector<double>> aux_mapper(const xbot_msgs::CustomState& aux_sig);

    };

    class IqEstimator
    {

      public:

        IqEstimator(Eigen::VectorXd K_t,
                    Eigen::VectorXd K_d0, Eigen::VectorXd K_d1,
                    Eigen::VectorXd rot_MoI,
                    Eigen::VectorXd red_ratio,
                    double tanh_coeff = 10.0);

        IqEstimator();

        IqEstimator(Eigen::VectorXd K_t);

        void set_current_state(Eigen::VectorXd q_dot, Eigen::VectorXd q_ddot, Eigen::VectorXd tau);

        void get_iq_estimate(std::vector<float>& iq_est);
        void get_iq_estimate(Eigen::VectorXd& iq_est);
        void get_iq_estimate(std::vector<float>& iq_est,
                             Eigen::VectorXd K_d0, Eigen::VectorXd K_d1);
        void get_iq_estimate(Eigen::VectorXd& iq_est,
                             Eigen::VectorXd K_d0, Eigen::VectorXd K_d1);

        void get_tau_link(Eigen::VectorXd& tau);
        void get_tau_friction(Eigen::VectorXd& tau_friction);
        void get_q_ddot(Eigen::VectorXd& q_ddot);

      private:

        Eigen::VectorXd _K_t, _K_d0, _K_d1, _rot_MoI, _red_ratio,
                        _iq_est, _tau_l, _tau_friction;

        Eigen::VectorXd _q_dot, _q_ddot, _tau;

        double _tanh_coeff = 10.0;

        int _n_jnts;

        void compute_iq_estimates();


    };

    class IqCalib
    {

      public:

        IqCalib();

        IqCalib(int window_size,
                Eigen::VectorXd K_t,
                Eigen::VectorXd rot_MoI,
                Eigen::VectorXd red_ratio,
                double tanh_coeff = 10.0,
                bool verbose = false);

        void add_sample(Eigen::VectorXd q_dot,
                   Eigen::VectorXd q_ddot,
                   Eigen::VectorXd iq,
                   Eigen::VectorXd tau);

        void get_current_optimal_Kd(Eigen::VectorXd& Kd0_opt,
                               Eigen::VectorXd& Kd1_opt);

        void get_current_tau_total(Eigen::VectorXd& tau_total);
        void get_current_tau_friction(Eigen::VectorXd& tau_friction);
        void get_current_alpha(Eigen::VectorXd& alpha_d0, Eigen::VectorXd& alpha_d1);

      private:

        bool _verbose = false; // whether to print info messages

        int _window_size = 300; // number of samples which will be retained
                                // and used to solve the calibration problem

        int _n_jnts = - 1; // dimension of the input signal ( = number of joints
                           // on which calibration is run)

        double _tanh_coeff; // handtuned coefficient used to approximate the
                            // ideal sign() function with a C^{inf} hyperbolic tangent function.
                            // The higher tanh_coeff, the steeper the transition from -1 to 1 is.

        Eigen::VectorXd _alpha_d0, _alpha_d1; // we choose to model the tau_friction
                                              // (choice dictated by the observations
                                              // on the measured mismatch between tau_total and tau)
                                              // as
                                              // tau_friction = Kd0 * sign(q_dot) + Kd1 * q_dot
                                              // which is a friction torque made of a static component (Kd0 * sign(q_dot))
                                              // and a dynamic component ( Kd1 * q_dot)

        Eigen::MatrixXd _Alpha; // least square problem matrix (for all joints)
                                // _Alpha_i * _Kd_i = _tau_friction_i, where i is the i-th joint
                                // _Alpha is obtained stacking up [_alpha_d0, _alpha_d1]

        Eigen::VectorXd _tau_friction; // tau_friction is equal to the model mismatch
                                        // between tau_total and tau(measured torque). Ideally, these two quantities
                                        // coincide

        Eigen::VectorXd _tau_total; // total torque on the rotor (reported to
                                    // the link-side) which is computed (with measurements) as
                                    // tau_total = 1/red_ratio * (rot_MoI * q_ddot / red_ratio - K_t * iq)

        Eigen::VectorXd _Kd0, _Kd1; // current optimical iq model calibration coefficient
                                    // Kd0 -> static friction component
                                    // Kd1 -> dynamic friction component

        Eigen::VectorXd _q_dot, _q_ddot, _iq, _tau; // measurements necessary for the computation
                                                    // of the calibration coefficients

        Eigen::VectorXd _K_t,
                        _rot_MoI,
                        _red_ratio; // actuator paramters (supposed to be perfectly known in
                                    // advance)

        void shift_data(Eigen::VectorXd& data,
                        bool towards_back = true); // shift vector data towards the
                                                   // back of the data qeue
        void shift_data(Eigen::MatrixXd& data,
                        bool rowwise = true,
                        bool towards_back = true); // shift vector data towards the
                                                   // back of the data qeue

        void solve_iq_cal_QP(int jnt_index); // solve the calibration QP for a single joint

        void compute_alphad0();
        void compute_alphad1();
        void assemble_Alpha();

        void compute_tau_friction();

    };
}

namespace SignProcUtils{

    class NumDiff
    {
      public:

        NumDiff();

        NumDiff(int n_jnts, double dt, int order = 1);

        void add_sample(Eigen::VectorXd sample);

        void dot(Eigen::VectorXd& sample_dot, bool use_spline = false); // get an estimate of derivative
        // of the current sample

      private:

        // coefficient from n-th order num. differentiation
        // ordered from left to right, starting from the current sample

        // depending on _order, the estimate for the derivative of the samples
        // at the current time is given by the following backward differentiation equation:
        // v_dot(k) = { sum_{i=0}^{_order + 1} [ v(k -i) * k_n(_order) ) / _dt

        Eigen::VectorXd _k_n1, _k_n2, _k_n3,
                        _k_n4, _k_n5, _k_n6;

        Eigen::VectorXd _k_n; // assigned on the basis of the selected order of the estimate

        Eigen::MatrixXd _window_data;

        double _dt = 0.0; // assuming evenly spaced samples

        int _n_jnts, _order;

        std::vector<interp::spline> _spline_interp;
        Eigen::VectorXd _aux_time_vect;

    };

    class MovAvrgFilt
    {
      public:

        MovAvrgFilt();

        MovAvrgFilt(int n_jnts, double dt, int window_size = 10);
        MovAvrgFilt(int n_jnts, double dt, double cutoff_freq = 15);

        void add_sample(Eigen::VectorXd sample);

        void get(Eigen::VectorXd& filt_sample);

        void get_cutoff_freq(double& cutoff_f);
        void get_window_size(int& window_size);

      private:

        Eigen::MatrixXd _window_data;

        double _magic_number = 0.442946470689452340308369; // reference
        // @ https://dsp.stackexchange.com/questions/9966/what-is-the-cut-off-frequency-of-a-moving-average-filter

        int _n_jnts, _window_size;

        double _cutoff_freq = -1.0;

        double _samples_dt = -1.0;

        bool _is_first_run = true;

    };

    class AcfImpl
  {
      public:

          AcfImpl(int window_size = 1); // default constructor of the implementation of the renowed Ad Cazzum Filter

          // S -> matrix of samples (nd x Ns), where Ns are the number of samples and nd is the number of dimensions

          // method which stacks up filtered data from a
          // matrix of samples S
          // of dimension nd x Ns

          // constant D matrix : D * S(j, :).T = Delta(j), where Delta is the difference vector between samples (dim. Ns x 1)
          // of the data array j

          // the D is a Ns x Ns matrix with the following pattern:
          // D:=
          // [ 0  0  ............................  0]
          // |-1  1  0 ..........................  0|
          // | 0 -1  1  0 .......................  0|
          // | 0  0 -1  1  0 ....................  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0  .  .  .  .  .  .  .  .  .  .  .  0|
          // | 0 ........................ 0  . -1  1|

          // Delta(j) has the following pattern (j = 0,....., nd - 1)
          // Delta(j) :=
          // [             0              ]
          // | S(j, 1)     -  S(j, 0)     |
          // | S(j, 2)     -  S(j, 1)     |
          // | S(j, 3)     -  S(j, 2)     |
          // | S(j, .)     -  S(j, .)     |
          // | S(j, .)     -  S(j, .)     |
          // | S(j, .)     -  S(j, .)     |
          // | S(j, .)     -  S(j, .)     |
          // | S(j, .)     -  S(j, .)     |
          // | S(j, Ns - 2) -  S(j, Ns - 3) |
          // [ S(j, Ns - 1) -  S(j, Ns - 2) ]atanh

          // and DELTA is a matrix of dimension nd x Ns
          // DELTA :=
          // [      Delta(0).T ]
          // |      Delta(1).T |
          // |      Delta(.).T |
          // | Delta(nd - 2).T |
          // | Delta(nd - 1).T |

          // with these definitions,
          // S(j, :) * D.T =  Delta(j).T
          // --> S * D.T = DELTA

          // Mu is a weight vector for the filtering window indicating how much each
          // sample in S(j, :) is influenced by previous one (one would want something like an hyperbolic function
          // so that older samples are weighted less)
          // Mu:=
          // [     Mu(0)  ]
          // |     Mu(1)  |
          // |     Mu(h)  |
          // |   Mu(nw-2)  |
          // [   Mu(nw-1)  ]
          // where Mu(0) weights the difference S(j, k) - S(j, k - 1)
          // Mu(1) weights the difference S(j, k - 1) - S(j, k - 2)
          // and so on and so forth

          // we also need to define the vector Dt which holds the time differences between samples
          // (we assume that all trains of data S(j, :) are sampled with the same rate):
          // Dt pattern (dimension (Ns - 1) x 1 ):
          // [     0     ]
          // | Dt( 0 )   |
          // | Dt( 1 )   |
          // | Dt( . )   |
          // [ Dt( Ns - 2)]

          // let's also define the matrix XI of dimension n x Ns
          // XI is a matrix of dimensions n x Ns, where n is the order of the filter or, equivalently, the window length

          // XI has the following pattern:

          // [0   Mu[0]/Dt(1)     Mu[0]/Dt(2)    Mu[0]/Dt(3) . .        Mu[0]/Dt(nw - 1)    .         Mu[0]/Dt(k)    .         Mu[0]/Dt(Ns - 1)]
          // |.             0     Mu[1]/Dt(2)    Mu[1]/Dt(3) . .        Mu[1]/Dt(nw - 1)    .         Mu[1]/Dt(k)    .         Mu[1]/Dt(Ns - 1)|
          // |.             0               0    Mu[2]/Dt(3) . .        Mu[2]/Dt(nw - 1)    .         Mu[2]/Dt(k)    .         Mu[2]/Dt(Ns - 1)|
          // |.             .               .              . . .        Mu[.]/Dt(nw - 1)    .         Mu[.]/Dt(k)    .         Mu[.]/Dt(Ns - 1)|
          // |.             .               0              0 . .   Mu[nw - 2]/Dt(nw - 1)    .    Mu[nw - 2]/Dt(k)    .    Mu[nw - 2]/Dt(Ns - 1)|
          // [0             .               0              0 . .   Mu[nw - 1]/Dt(nw - 1)    .    Mu[nw - 1]/Dt(k)    .    Mu[nw - 1]/Dt(Ns - 1)]

          // S * D.T = DELTA

          // W is the "window matrix" of dimension Ns x Ns which has the following pattern:

          // [1 0 ............................0]
          // |1 1 0 ..........................0|
          // |1 1 1 0 ........................0|
          // |1 . . 1 0 ......................0|
          // |1 . . . 1 0 ....................0|
          // |0 1 . . . 1 0 ..................0|
          // |0 0 1 . . . 1 0 ................0|
          // [. . . . . . . . . . . . . . . . 0]
          // [. . . . . . . . . . . . . . . . 0]
          // [. . . . . . . . . . . . . . . . 0]
          // [. . . . . . . . . . . . . . . . 0]
          // [. . . . . . . . . . . . . . . . 0]
          // [0.............. 1 . . . . 1 0 0 0]
          // [0................ 1 . . . . 1 0 0]
          // [0.................. 1 . . . . 1 0]
          // [0.....................1 . . . . 1]

  };

}

#endif
