#ifndef PLUGIN_UTILS_H
#define PLUGIN_UTILS_H

#include <math.h> 

#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
#include <vector>
#include <fstream>

#include <stdexcept>

namespace plugin_utils{

    class PeisekahTrans
    {
        public:

            PeisekahTrans(); // default constructor

            PeisekahTrans(Eigen::VectorXd start_point, Eigen::VectorXd end_point, double exec_time, double dt)
            : _start_point{start_point}, _end_point{end_point}, _exec_time{exec_time}, _dt{dt}
            {

                check_input_dim();

                compute_traj();

            }

            Eigen::VectorXd eval_at(int node)
            {

                return _traj.col(node);

            }

            double get_exec_time()
            {
                return _exec_time;
            }

            double get_traj_dt()
            {
                return _dt;
            }

            double get_n_nodes()
            {
                return _n_nodes;
            }

            double get_n_dim()
            {
                return _n_dim;
            }

        private:

            Eigen::VectorXd _start_point, _end_point;
            Eigen::MatrixXd _traj;

            double _exec_time, _dt;
            int _n_nodes, _n_dim;

            void check_input_dim()
            {   
                int start_size = _start_point.size(); 
                int end_size = _end_point.size();
                if ( start_size != end_size)
                { 

                    std::string exception = std::string("The starting point and end point dimensions do not match: ") + 
                                            std::to_string(start_size) + std::string(" VS ") + std::to_string(end_size) + std::string("\n");
                                            
                    throw std::invalid_argument(exception);
                    
                }
            }

            void rate_adapter()
            {
                double dt_des = _dt;

                double n_int_aux = floor(_exec_time / dt_des);

                double dt1 = _exec_time / n_int_aux;
                double dt2 = _exec_time / (n_int_aux + 1);
                double abs_diff1 = abs(dt1);
                double abs_diff2 = abs(dt2);

                if (abs_diff1 < abs_diff2)
                { 
                    _n_nodes = n_int_aux + 1;
                    _dt = dt1;
                }
                else
                {
                    _n_nodes = n_int_aux + 2;  
                    _dt = dt2;
                }
            }

            double compute_peisekah_val(int node, int n_nodes, double start_point, double end_point)
            {

                double common_part_traj = (126.0 * pow(node/(n_nodes - 1), 5) - 420.0 * pow(node/(n_nodes - 1), 6) + 540.0 * pow(node/(n_nodes - 1), 7) - 315.0 * pow(node/(n_nodes - 1), 8) + 70.0 * pow(node/(n_nodes - 1), 9));

                auto value = start_point + (end_point - start_point) *  common_part_traj;

                return value;

            }

            void compute_traj()
            {
                rate_adapter(); // adapt _dt and _n_nodes to match the required _exec_time

                for (int k = 0; k < _n_dim; k++)
                { // loop through joints (rows)

                    for (int i = 0; i < _n_nodes; i++)
                    { // loop through samples (columns)

                        _traj(k, i) = compute_peisekah_val(i, _n_nodes, _start_point(k), _end_point(k));

                    }

                }
                
            }

    };

    class TrajLinInterp
    {
        public:

            TrajLinInterp(); // default constructor

            TrajLinInterp(Eigen::VectorXd sample_time, Eigen::MatrixXd input_traj, int interp_dir = 1)
            : _sample_times{sample_time}, _traj{input_traj}, _interp_dir{interp_dir}
            {
                
                check_dim_match(); // at this point, it is guaranteed that dimensions along the interpolation axis (which now is for sure valid) match
  
                _n_dims = (_interp_dir == 1) ? _traj.rows(): _traj.cols();

            }

            Eigen::MatrixXd eval_at(Eigen::VectorXd& interp_times)
            {
                // check interp_times is within _sample_times
                check_time_vector(interp_times);

                int n_int_samples = interp_times.size();
                Eigen::MatrixXd inter_mat = Eigen::MatrixXd::Zero(_n_dims, n_int_samples);

                for (int i = 0; i < _n_dims; i++)
                {

                    if (_interp_dir == 1)
                    { //  row-wise

                        inter_mat.row(i) = interp_1d(interp_times, i);

                    }
                    else
                    { // col-wise

                        inter_mat.col(i) = interp_1d(interp_times, i);

                    }

                }

                return inter_mat;

            }

        private: 

            Eigen::MatrixXd _traj; // will be casted to the right type upon constructor call
            Eigen::VectorXd _sample_times;

            int _n_samples, _n_dims;

            int _interp_dir;

            void check_time_vector(Eigen::VectorXd interp_times)
            {
                int n_int_samples = interp_times.size();

                if ( (interp_times(0) <  _sample_times(0)) || (interp_times(n_int_samples - 1) > _sample_times(_n_samples)) )
                { // checking interp_times is actually within _sample_times 

                    std::string exception = std::string("The provided interpolation array ") + 
                                            std::string("[") + std::to_string(interp_times(0)) + std::string(", ") + std::to_string(interp_times(n_int_samples - 1)) + std::string("]") +
                                            std::string(" is outside the trajectory time array ") +
                                            std::string("[") + std::to_string(_sample_times(0)) + std::string(", ") + std::to_string(_sample_times(_n_samples)) + std::string("]\n");

                    throw std::invalid_argument(exception);
                    
                }

            }

            int get_traj_n_samples()
            {

                if (_interp_dir == 0)
                { // colum-wise
                    int sample_number = _traj.rows();

                    return sample_number; 
                }
                else
                {
                    if (_interp_dir == 1)
                    { // row-wise
                        int sample_number = _traj.cols();
                    
                        return sample_number; 
                    }
                    else
                    {
                        std::string exception = "Invalid interpolation direction " + std::to_string(_interp_dir) + "provided. " +
                                                "Allowed values are 0(row-wise) and 1(colum-wise)\n";

                        _interp_dir = -1;

                        throw std::invalid_argument(exception);

                    }
                }                

            }

            void check_dim_match()
            {
                _n_samples = get_traj_n_samples();
            
                if ( (_n_samples != _sample_times.rows()) || (_n_samples != _sample_times.cols()) )
                { // checking that time vector and trajectory dimensions match 

                    std::string exception = std::string("Time array and trajectory dimension do not match. ") + 
                                            std::string("They are, respectively,") + 
                                            std::string(" (") + std::to_string(_sample_times.rows()) + std::string(", ") + std::to_string(_sample_times.cols()) + std::string(")") +
                                            std::string("and") + 
                                            std::string(" (") + std::to_string(_traj.rows()) + ", " + std::to_string(_traj.cols()) + std::string(")\n");

                    throw std::invalid_argument(exception);

                }
            }
            
            Eigen::VectorXd interp_1d(Eigen::VectorXd& interp_times, int dim_index)
            {   
                int n_int_samples = interp_times.size();

                Eigen::VectorXd interp_vect = Eigen::VectorXd::Zero(n_int_samples);
                
                for (int i = 0; i < n_int_samples; i++)
                {
                    std::vector<int> points_index = get_closest_points(interp_times(i));

                    double interval_dt = _sample_times(points_index[1]) - _sample_times(points_index[0]);

                    double t_norm = interp_times(i) - _sample_times(points_index[0]);

                    interp_vect(i) = interp_0d(t_norm, interval_dt, points_index, dim_index);
                }

                return interp_vect;
            }

            double interp_0d(double t_norm, double interval_dt, std::vector<int> points_index, int dim_index)
            {   
                double interp_val;

                if (_interp_dir == 1)
                { // row-wise

                    interp_val = _traj(dim_index, points_index[0]) + t_norm / interval_dt * (_traj(dim_index, points_index[1]) - _traj(dim_index, points_index[0]));

                } // col-wise
                else{

                    interp_val = _traj(points_index[0], dim_index) + t_norm / interval_dt * (_traj(points_index[1], dim_index) - _traj(points_index[0], dim_index));

                }
                
                return interp_val;
            }

            std::vector<int> get_closest_points(double inter_time)
            {
                std::vector<int> closest_pnts_indexes(-1, -1); // initialize indeces to -1 
        
                Eigen::VectorXd diff_array = Eigen::VectorXd::Zero(_n_samples); 

                for (int i = 0; i < _n_samples; i++)
                {
                    diff_array(i) = abs(inter_time - _sample_times(i)); 
                }
                int index_aux; 
                diff_array.minCoeff(&index_aux);

                if (diff_array[index_aux] < 0)
                {
                    closest_pnts_indexes[0] = index_aux - 1;
                    closest_pnts_indexes[1] = index_aux;
                }
                else
                {
                    closest_pnts_indexes[0] = index_aux;
                    closest_pnts_indexes[1] = index_aux + 1;
                }

                return closest_pnts_indexes;
            }
    };

    class TrajLoader
    {
        public:

            TrajLoader(); // default constructor

            TrajLoader(std::string data_path, bool column_major = true)
            :_data_path{data_path}, _column_major_order{column_major}
            {

                std::string extension = get_file_extension(data_path);

                if (extension == "")
                { // no extension --> this class will assume the user wants to read from csv files
                    
                    load_data_from_csv(data_path);

                }
                else if(extension == "mat")
                { // load from .mat database

                    load_data_from_mat(data_path);

                }
                else
                { // unsupported extension 

                    throw std::runtime_error(std::string("Unsupported extension ") + extension + std::string("provided.\n"));

                }

                check_loaded_data_dims();
                
                _n_nodes = get_n_samples(_q_p);
                _n_jnts = get_n_jnts(_q_p);

                _sample_times = Eigen::VectorXd::Zero(_n_nodes);
                for (int i = 0; i < (_n_nodes - 1); i++)
                {
                    _sample_times(i + 1) = _sample_times(i) + _dt_opt(i);
                }

                int interp_dir = (_column_major_order) ? 1 : 0;
                opt_traj.emplace(_q_p_name, TrajLinInterp(_sample_times, _q_p, interp_dir));
                opt_traj.emplace(_q_p_dot_name, TrajLinInterp(_sample_times, _q_p_dot, interp_dir));
                opt_traj.emplace(_efforts_name, TrajLinInterp(_sample_times.head(_n_nodes - 1), _tau, interp_dir));          
                
            }

            Eigen::MatrixXd read_data_from_csv(std::string data_path)
            { // keep it public so it can also be used outside class instances
                using namespace Eigen;
                using namespace std;

                vector<double> matrixEntries;
            
                // in this object we store the data from the matrix
                ifstream matrixDataFile(data_path);
            
                // this variable is used to store the row of the matrix that contains commas 
                string matrixRowString;
            
                // this variable is used to store the matrix entry;
                string matrixEntry;
            
                // this variable is used to track the number of rows
                int matrixRowNumber = 0;
            
            
                while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
                {
                    stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
            
                    while (getline(matrixRowStringStream, matrixEntry, ' ')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
                    {
                        matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
                    }
                    matrixRowNumber++; //update the column numbers
                }
            
                // here we convet the vector variable into the matrix and return the resulting object, 
                // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
                return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
            
            }

            int get_n_jnts()
            {
                return _n_jnts;
            }

            int get_n_nodes()
            {
                return _n_nodes;
            }

            Eigen::MatrixXd eval_q_p_at(Eigen::VectorXd& times)
            {
                return opt_traj[_q_p_name].eval_at(times);
            }

            Eigen::MatrixXd eval_q_p_dot_at(Eigen::VectorXd& times)
            {
                return opt_traj[_q_p_dot_name].eval_at(times);
            }

            Eigen::MatrixXd eval_tau_at(Eigen::VectorXd& times)
            {
                return opt_traj[_efforts_name].eval_at(times);
            }

            
        private:

            std::string _data_path;
            bool _column_major_order; 

            std::string _q_p_name = "q_p";
            std::string _q_p_dot_name = "q_p_dot";
            std::string _efforts_name = "tau";
            std::string _dt_name = "dt_opt";

            Eigen::MatrixXd _q_p;
            Eigen::MatrixXd _q_p_dot;
            Eigen::MatrixXd _tau;
            Eigen::VectorXd _dt_opt, _sample_times;

            int _n_nodes, _n_jnts;

            std::map<std::string, TrajLinInterp> opt_traj; 

            std::string get_file_extension(std::string file)
            {
                std::vector<std::string> token_list;
                boost::split(token_list, file, [](char c){return c == '.';});
                
                if(token_list.size() > 1)
                {
                    return token_list.back();
                }
                
                return "";
            }

            int get_n_jnts(Eigen::MatrixXd& mat)
            {
                if(_column_major_order)
                {
                    return mat.rows();
                }
                else
                {
                    return mat.cols();
                }
            }
            
            int get_n_samples(Eigen::MatrixXd& mat)
            {
                if(_column_major_order)
                {
                    return mat.cols();
                }
                else
                {
                    return mat.rows();
                }
            }

            void check_loaded_data_dims()
            {
                if ( !( ( get_n_jnts(_q_p) == get_n_jnts(_q_p_dot) ) && (get_n_jnts(_q_p_dot) == get_n_jnts(_tau)) ) )
                { // check number of joints consistency between data

                    throw std::invalid_argument(std::string("The number of rows (i.e. joints) of the loaded data does not match!\n"));

                }

                if ( !( (get_n_samples(_q_p) == get_n_samples(_q_p_dot)) && (get_n_samples(_q_p_dot) == (get_n_samples(_tau) + 1 )) ) )
                { // check cols (torque matri)

                    throw std::invalid_argument(std::string("The number of columns (i.e. samples) of the loaded data does not match!\n"));

                }

                if (get_n_samples(_tau) != _dt_opt.size())
                {
                    throw std::invalid_argument(std::string("The size of the loaded dt vector does not match the other data!.\n"));
                }

            }

            void load_data_from_csv(std::string data_path)
            {

                std::string q_p_path = data_path + _q_p_name + std::string(".csv");
                std::string q_p_dot_path = data_path + _q_p_dot_name + std::string(".csv");
                std::string tau_path = data_path + _efforts_name + std::string(".csv");
                std::string dt_path = data_path + _dt_name + std::string(".csv");

                _q_p = read_data_from_csv(q_p_path);
                _q_p_dot = read_data_from_csv(q_p_dot_path);
                _tau = read_data_from_csv(tau_path);
                _dt_opt = read_data_from_csv(dt_path);

                if (_q_p.size() == 0)
                { // reading failed    
                    throw std::runtime_error(std::string("Failed to find q_p at ") + q_p_path);
                }
                if (_q_p_dot.size() == 0)
                { // reading failed    
                    throw std::runtime_error(std::string("Failed to find q_p_dot at ") + q_p_dot_path);
                }
                if (_tau.size() == 0)
                { // reading failed    
                    throw std::runtime_error(std::string("Failed to find tau at ") + tau_path);
                }
                if (_dt_opt.size() == 0)
                { // reading failed    
                    throw std::runtime_error(std::string("Failed to find dt_opt at ") + dt_path);
                }


            }

            void load_data_from_mat(std::string math_path)
            {
                
                throw std::invalid_argument(std::string("Reading from mat databases is not yet supported !! \n")); // to be removed upon new MatLogger2 merge

                // XBot::MatLogger2::Options opts;
                // opts.load_file_from_path = true; // enable reading
                // XBot::MatLogger2::Ptr _load_logger = XBot::MatLogger2::MakeLogger(math_path, opts);

                // int slices; // not needed, used just to call the method properly 
                // bool q_p_read_ok = _load_logger->readvar(_q_p_name, _q_p, slices);
                // bool q_p_dot_read_ok = _load_logger->readvar(_q_p_dot_name, _q_p_dot, slices);
                // bool tau_read_ok = _load_logger->readvar(_efforts_name, _tau, slices);
                // bool dt_read_ok = _load_logger->readvar(_dt_name, _dt_opt, slices);

                // if (!q_p_read_ok)
                // { // reading failed    
                //     throw std::runtime_error(std::string("Failed to find q_p from mat database at ") + math_path);
                // }
                // if (!q_p_dot_read_ok)
                // { // reading failed    
                //     throw std::runtime_error(std::string("Failed to find q_p_dot from mat database at ") + math_path);
                // }
                // if (!tau_read_ok)
                // { // reading failed    
                //     throw std::runtime_error(std::string("Failed to find tau from mat database at ") + math_path);
                // }
                // if (!dt_read_ok)
                // { // reading failed    
                //     throw std::runtime_error(std::string("Failed to find dt_opt from mat database at ") + math_path);
                // }

            }


    };
}

#endif