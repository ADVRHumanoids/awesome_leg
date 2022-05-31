#ifndef PLUGIN_UTILS_H
#define PLUGIN_UTILS_H

#include <math.h> 

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>

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


}

#endif