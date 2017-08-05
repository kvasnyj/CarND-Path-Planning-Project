//
// Created by kvasnyj on 4/8/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>

using namespace std;

class planner {
public:
    void straight(vector<double> &next_x_vals, vector<double> &next_y_vals, double car_x, double car_y, double car_yaw);

    void circle(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                vector<double> &previous_path_y, double car_x, double car_y, double car_yaw);

    void FollowingWP(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                vector<double> &previous_path_y, vector<double> &telemetry);
};


#endif //PATH_PLANNING_PLANNER_H
