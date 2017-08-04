//
// Created by kvasnyj on 4/8/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include <math.h>

using namespace std;

class planner {
public:
    static constexpr double pi() { return M_PI; }
    void straight(vector<double> &next_x_vals, vector<double> &next_y_vals, double car_x, double car_y, double car_yaw);
    void circle(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x, vector<double> &previous_path_y, double car_x, double car_y, double car_yaw);
    };


#endif //PATH_PLANNING_PLANNER_H
