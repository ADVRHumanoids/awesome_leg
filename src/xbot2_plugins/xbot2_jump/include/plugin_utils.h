#ifndef PLUGIN_UTILS_H
#define PLUGIN_UTILS_H

#include <math.h> 

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>

#include <stdexcept>

namespace plugin_utils{



    bool compute_peisekah_val(double time, double exec_time, double start_point, double end_point)
    {
        double common_part_traj = (126.0 * pow(time/exec_time, 5) - 420.0 * pow(time/exec_time, 6) + 540.0 * pow(time/exec_time, 7) - 315.0 * pow(time/exec_time, 8) + 70.0 * pow(time/exec_time, 9));

        auto value = start_point + (end_point - start_point) *  common_part_traj;

        return value;
    }


    Eigen::MatrixXd openData(std::string fileToOpen)
    {
        using namespace Eigen;
        using namespace std;

        vector<double> matrixEntries;
    
        // in this object we store the data from the matrix
        ifstream matrixDataFile(fileToOpen);
    
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

    class TrajLinInterp
    {
        public:

            TrajLinInterp(Eigen::VectorXd sample_time, Eigen::MatrixXd input_traj, int interp_dir = 1)
            : _sample_times{sample_time}, _traj{input_traj}, _interp_dir{interp_dir}
            {
                
                check_dim_match(); // at this point, it is guaranteed that dimensions along the interpolation axis (which now is for sure valid) match
  
                _n_dims = (_interp_dir == 1) ? _traj.rows(): _traj.cols();

            }

            Eigen::MatrixXd eval_at(Eigen::VectorXd interp_times)
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


}

#endif