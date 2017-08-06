//
// Created by kvasnyj on 4/8/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include <string>
#include "waypoints.h"

using namespace std;

class planner {
public:
    struct jerk_result {
        vector<double> path_x;
        vector<double> path_y;
        double last_s;
        double last_d;
    };

    double cost_lane_change_front;
    double cost_lane_change_rear;
    double cost_straight;
    double cost_max;


    double max_speed;
    double min_speed;

    double path_plan_seconds;
    double delta_t;

    int i_x;
    int i_y;
    int i_s;
    int i_d;
    int i_yaw;
    int i_speed;
    int i_lane;

    vector<vector<double>> other_cars;
    vector<double> prev_state;

    waypoints wp;

    planner();

    void straight(vector<double> &next_x_vals, vector<double> &next_y_vals, double car_x, double car_y, double car_yaw);

    void circle(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                vector<double> &previous_path_y, double car_x, double car_y, double car_yaw);

    void FollowingWP(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                     vector<double> &previous_path_y, vector<double> &telemetry, vector<vector<double> > sensor_fusion);

    vector<double> computeMinimumJerk(vector<double> start, vector<double> end, double max_time, double time_inc);

    vector<double> distanceToClosestCar(double car_s, int car_l, bool inFront);

    double costOfLaneChange(double car_s, int car_l, int direction);

    double costOfStraightCourse(double car_s, int car_l);

    vector<double> determineNewStraightCourseSetpoints(double car_s, int car_l, double car_speed);

    vector<double> determineNewCourseSetpoints(double car_s, int car_l, double car_speed, int direction);

    void fill_other_cars(vector<vector<double>> sensor_fusion);

    int lowestCostAction(double car_s, int car_l);

    jerk_result computeMinimumJerkMapPath(vector<double> new_setpoints,
                                          vector<double> map_waypoints_s,
                                          vector<double> map_waypoints_x,
                                          vector<double> map_waypoints_y);
};


#endif //PATH_PLANNING_PLANNER_H
