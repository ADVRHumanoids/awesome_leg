#ifndef PLUGIN_UTILS_H
#define PLUGIN_UTILS_H

#include <math.h> 

namespace plugin_utils{

    bool compute_peisekah_val(double time, double exec_time, double start_point, double end_point)
    {
        double common_part_traj = (126.0 * pow(time/exec_time, 5) - 420.0 * pow(time/exec_time, 6) + 540.0 * pow(time/exec_time, 7) - 315.0 * pow(time/exec_time, 8) + 70.0 * pow(time/exec_time, 9));

        auto value = start_point + (end_point - start_point) *  common_part_traj;

        return value;
    }


}

#endif