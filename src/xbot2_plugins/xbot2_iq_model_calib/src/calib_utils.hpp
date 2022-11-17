#ifndef CALIB_UTILS_H
#define CALIB_UTILS_H

#include <math.h> 

 #include <Eigen/Dense>
 #include <Eigen/Core>
#include <Eigen/Eigen>

#include <vector>
#include <map>

namespace calib_utils{

    class AcfImpl
    {
        public:

            AcfImpl(); // default constructor of the implementation of the Ad Cazzum Filter

            AcfImpl();

            // -> we need:
            // method for filtering row of data (size N)
            // method which stacks up filtered data from a matrix of samples S of dimension nd x N
            // -> what is needed for the "single row" filtering part:
            // initialization of D matrix (always the same) : D * S(i, :).T = Delta, where Delta is the difference vector between samples (dim. N x 1), with first el. set to 0

            // the D is a N x N matrix with the following pattern:

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

            // Delta is a vector with the following pattern (j = 0,....., nd - 1)

            // [             0              ]
            // | S(j, 1)     -  S(j, 0)     |
            // | S(j, 2)     -  S(j, 1)     |
            // | S(j, 3)     -  S(j, 2)     |
            // | S(j, .)     -  S(j, .)     |
            // | S(j, .)     -  S(j, .)     |
            // | S(j, .)     -  S(j, .)     |
            // | S(j, .)     -  S(j, .)     |
            // | S(j, .)     -  S(j, .)     |
            // | S(j, N - 2) -  S(j, N - 3) |
            // [ S(j, N - 1) -  S(j, N - 2) ]

            // Mu is a weight vector for the filtering window indicating how much each filtered sample is influenced by previous samples (one would want something like an hyperbolic function

            // [ Mu( 0 ) ]
            // | Mu( 1 ) |
            // | Mu( h ) |
            // | Mu(n-2) |
            // [ Mu(n-1) ]

            // where mu(0) weights the difference between the k-th and the (k-th - 1) sample

            // we also need a vector Dt of time differences between the samples:
            // Dt pattern:
            // [     0     ]
            // | Dt( 0 )   |
            // | Dt( 1 )   |
            // | Dt( . )   |
            // [ Dt( N - 2)]

            // let's also define the vector Xi of dimension 1 x N
            // Xi pattern:
            // Mu[h] * [1.0/Dt(k - 0)]

            // XI is a matrix of dimensions n x N, where n is the order of the filter or, equivalently, the window length

            // XI has the following pattern:

            // [0   Mu[0]/Dt(1)     Mu[0]/Dt(2)    Mu[0]/Dt(3) . .       Mu[0]/Dt(n - 1)    .       Mu[0]/Dt(k)    .        Mu[0]/Dt(N - 1)]
            // |.             0     Mu[1]/Dt(2)    Mu[1]/Dt(3) . .       Mu[1]/Dt(n - 1)    .       Mu[1]/Dt(k)    .        Mu[1]/Dt(N - 1)|
            // |.             0               0    Mu[2]/Dt(3) . .       Mu[2]/Dt(n - 1)    .       Mu[2]/Dt(k)    .        Mu[2]/Dt(N - 1)|
            // |.             .               .              . . .       Mu[.]/Dt(n - 1)    .       Mu[.]/Dt(k)    .        Mu[.]/Dt(N - 1)|
            // |.             .               0              0 . .   Mu[n - 2]/Dt(n - 1)    .   Mu[n - 2]/Dt(k)    .    Mu[n - 2]/Dt(N - 1)|
            // [0             .               0              0 . .   Mu[n - 1]/Dt(n - 1)    .   Mu[n - 1]/Dt(k)    .    Mu[n - 1]/Dt(N - 1)]

            // W is the "window matrix" of dimension N x N which has the following pattern:

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



            // matrix of samples ->
            
            Eigen::VectorXd eval_at(int node);

            double compute_peisekah_val(double phase, double start_point, double end_point);
            Eigen::VectorXd compute_peisekah_vect_val(double phase, Eigen::MatrixXd start_point,  Eigen::MatrixXd end_point);

            double get_exec_time();
            double get_traj_dt();
            double get_n_nodes();
            double get_n_dim();
            Eigen::MatrixXd get_traj();

        private:

            Eigen::VectorXd _start_point, _end_point;
            Eigen::MatrixXd _traj;

            double _exec_time, _dt;
            int _n_nodes, _n_dim;

            void check_input_dim();
            void rate_adapter();
            void compute_traj();

    };
}

#endif
