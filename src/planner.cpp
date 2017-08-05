//
// Created by kvasnyj on 4/8/17.
//
#include <vector>
#include "planner.h"
#include "MathHelper.h"
#include <math.h>

using namespace std;



void planner::straight(vector<double> &next_x_vals, vector<double> &next_y_vals, double car_x, double car_y,
                       double car_yaw) {
    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
        next_x_vals.push_back(car_x + (dist_inc * i) * cos(MathHelper::deg2rad(car_yaw)));
        next_y_vals.push_back(car_y + (dist_inc * i) * sin(MathHelper::deg2rad(car_yaw)));
    }
}

void planner::circle(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                     vector<double> &previous_path_y, double car_x, double car_y, double car_yaw) {
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size();

    for (int i = 0; i < path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if (path_size == 0) {
        pos_x = car_x;
        pos_y = car_y;
        angle = MathHelper::deg2rad(car_yaw);
    } else {
        pos_x = previous_path_x[path_size - 1];
        pos_y = previous_path_y[path_size - 1];

        double pos_x2 = previous_path_x[path_size - 2];
        double pos_y2 = previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; i++) {
        next_x_vals.push_back(pos_x + (dist_inc) * cos(angle + (i + 1) * (MathHelper::pi() / 100)));
        next_y_vals.push_back(pos_y + (dist_inc) * sin(angle + (i + 1) * (MathHelper::pi() / 100)));
        pos_x += (dist_inc) * cos(angle + (i + 1) * (MathHelper::pi() / 100));
        pos_y += (dist_inc) * sin(angle + (i + 1) * (MathHelper::pi() / 100));
    }

}

void planner::FollowingWP(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                vector<double> &previous_path_y, vector<double> &telemetry)
{
    double car_x = telemetry[0];
    double car_y = telemetry[1];
    double car_s = telemetry[2];
    double car_d = telemetry[3];
    double car_yaw = telemetry[4];
    double car_speed = telemetry[5];
}
