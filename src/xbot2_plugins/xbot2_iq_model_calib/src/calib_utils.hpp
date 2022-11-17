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

            AcfImpl(); // default constructor of the implementation of the renowed Ad Cazzum Filter

            AcfImpl();

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
            // [ S(j, Ns - 1) -  S(j, Ns - 2) ]

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
