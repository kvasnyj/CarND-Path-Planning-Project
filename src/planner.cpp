//
// Created by kvasnyj on 4/8/17.
//
#include <vector>
#include "planner.h"
#include "MathHelper.h"
#include <math.h>
#include <iostream>
#include <map>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

planner::planner() {
    prev_state.reserve(3);

    cost_lane_change_front = 1;
    cost_lane_change_rear = 0.5;
    cost_straight = 1.0;
    cost_max = 10000.0;

    max_speed = 20.0;
    min_speed = 10.0;

    path_plan_seconds = 2.5;
    delta_t = 0.02;

    i_x = 0;
    i_y = 1;
    i_s = 2;
    i_d = 3;
    i_yaw = 4;
    i_speed = 5;
    i_lane = 6;
}

void planner::FollowingWP(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> &previous_path_x,
                          vector<double> &previous_path_y, vector<double> &telemetry,
                          vector<vector<double> > sensor_fusion) {
    telemetry[planner::i_lane] = wp.D2Lane(telemetry[i_d]);

    int path_size = previous_path_x.size();

    vector<double> setpoints(6);

    if (path_size == 0) {
        fill_other_cars(sensor_fusion);

        setpoints = determineNewStraightCourseSetpoints(telemetry[i_s], telemetry[planner::i_lane],
                                                        telemetry[planner::i_speed]);
        jerk_result jerk = computeMinimumJerkMapPath(setpoints,
                                                     wp.map_waypoints_s,
                                                     wp.map_waypoints_x,
                                                     wp.map_waypoints_y);
        prev_state[0] = jerk.last_s;
        prev_state[1] = jerk.last_d;
        prev_state[2] = setpoints[3];

        next_x_vals = jerk.path_x;
        next_y_vals = jerk.path_y;
    } else if (path_size < 15) {
        fill_other_cars(sensor_fusion);

        double car_s = prev_state[0];
        double car_d = prev_state[1];
        double car_speed = prev_state[2];
        int car_l = wp.D2Lane(car_d);

        int action = lowestCostAction(car_s, car_l);
        if (action < 0)
            setpoints = determineNewCourseSetpoints(car_s, car_l, car_speed, -1);
        else if (action == 0)
            setpoints = determineNewStraightCourseSetpoints(car_s, car_l, car_speed);
        else if (action > 0)
            setpoints = determineNewCourseSetpoints(car_s, car_l, car_speed, 1);

        vector<double> path_x = previous_path_x;
        vector<double> path_y = previous_path_y;
        jerk_result jerk = computeMinimumJerkMapPath(setpoints,
                                                     wp.map_waypoints_s,
                                                     wp.map_waypoints_x,
                                                     wp.map_waypoints_y);
        prev_state[0] = jerk.last_s;
        prev_state[1] = jerk.last_d;
        prev_state[2] = setpoints[3];
        for (int i = 0; i < jerk.path_x.size(); i++) {
            path_x.push_back(jerk.path_x[i]);
            path_y.push_back(jerk.path_y[i]);
        }
        next_x_vals = path_x;
        next_y_vals = path_y;
    } else {

        next_x_vals = previous_path_x;
        next_y_vals = previous_path_y;
    }

}

void planner::fill_other_cars(vector<vector<double>> sensor_fusion) {
    other_cars = {};
    for (int i = 0; i < sensor_fusion.size(); i++) {
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double speed = sqrt(vx * vx + vy * vy);

        vector<double> car(7);
        car[planner::i_s] = s;
        car[planner::i_d] = d;
        car[planner::i_speed] = speed;
        car[planner::i_lane] = wp.D2Lane(d);

        other_cars.push_back(car);
    }
}

vector<double> planner::distanceToClosestCar(double car_s, int car_l, bool inFront) {
    double closest = 100;
    double speed = 0;
    for (int i = 0; i < other_cars.size(); i++) {
        if (other_cars[i][i_lane] == car_l) {
            double diff = 0.0;
            if (inFront) diff = other_cars[i][i_s] - car_s;
            else diff = car_s - other_cars[i][i_s];

            if (diff >= 0.0 && diff < closest) {
                closest = diff;
                speed = other_cars[i][i_speed];
            }
        }
    }

    vector<double> result(2);
    result[0] = closest;
    result[1] = speed;
    return result;
}

double planner::costOfLaneChange(double car_s, int car_l, int direction) {
    int new_lane = car_l + direction;
    if (new_lane < 1 | new_lane > 3) {
        return cost_max;
    }

    double front_dist = distanceToClosestCar(car_s, new_lane, true)[0];
    double behind_dist = distanceToClosestCar(car_s, new_lane, false)[0];
    if (front_dist != 0.0 && behind_dist != 0.0) {
        return cost_lane_change_front / front_dist + cost_lane_change_rear / behind_dist;
    }
    return cost_max;
}

double planner::costOfStraightCourse(double car_s, int car_l) {
    double front_dist = distanceToClosestCar(car_s, car_l, true)[0];
    if (front_dist < 8)return 0; //if we too close to car, go straight
    if (front_dist != 0.0) {
        return cost_straight / front_dist;
    }
    return cost_max;
}

int planner::lowestCostAction(double car_s, int car_l) {
    double left_cost = costOfLaneChange(car_s, car_l, -1);
    double keep_cost = costOfStraightCourse(car_s, car_l);
    double right_cost = costOfLaneChange(car_s, car_l, 1);

    map<double, int> cost_map = {{left_cost,  -1},
                                 {keep_cost,  0},
                                 {right_cost, 1}};

    map<double, int>::iterator cost_map_iterator;
    cost_map_iterator = cost_map.begin();
    int action = cost_map_iterator->second;
    return action;
}

vector<double> planner::determineNewCourseSetpoints(double car_s, int car_l, double car_speed, int direction) {
    vector<double> setpoint(6);
    setpoint[0] = car_s;
    setpoint[1] = car_speed;
    setpoint[2] = car_s + path_plan_seconds * car_speed;
    setpoint[3] = car_speed*0.9;
    setpoint[4] = car_l;
    setpoint[5] = car_l + direction;

    return setpoint;
}

vector<double> planner::determineNewStraightCourseSetpoints(double car_s, int car_l, double car_speed) {
    double speed_start = car_speed;
    if (speed_start > max_speed) speed_start = max_speed;

    vector<double> car_in_front = distanceToClosestCar(car_s, car_l, true);
    double speed_end = speed_start * 1.5;
    if (speed_end < min_speed) speed_end = min_speed;

    if (car_in_front[0] < 25)
        speed_end = car_in_front[1];
    else if (car_in_front[0] < 20)
        speed_end = car_in_front[1] * 0.7;
    if (speed_end > max_speed) speed_end = max_speed;


    vector<double> setpoint(6);
    setpoint[0] = car_s;
    setpoint[1] = speed_start;
    setpoint[2] = car_s + path_plan_seconds * 0.5 * (speed_start + speed_end);
    setpoint[3] = speed_end;
    setpoint[4] = car_l;
    setpoint[5] = car_l;

    return setpoint;
}

vector<double> planner::computeMinimumJerk(vector<double> start, vector<double> end, double max_time, double time_inc) {
    MatrixXd A = MatrixXd(3, 3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t = max_time;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A << t3, t4, t5,
            3 * t2, 4 * t3, 5 * t4,
            6 * t, 12 * t2, 20 * t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
            end[1] - (start[1] + start[2] * t),
            end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    vector<double> result;
    for (double t = time_inc; t < max_time + 0.001; t += time_inc) {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
        result.push_back(r);
    }
    return result;
}

planner::jerk_result planner::computeMinimumJerkMapPath(vector<double> new_setpoints,
                                                        vector<double> map_waypoints_s,
                                                        vector<double> map_waypoints_x,
                                                        vector<double> map_waypoints_y) {
    double start_pos_s = new_setpoints[0];
    double start_vel_s = new_setpoints[1];
    double end_pos_s = new_setpoints[2];
    double end_vel_s = new_setpoints[3];

    double start_pos_d = wp.lane2D(new_setpoints[4]);
    double end_pos_d = wp.lane2D(new_setpoints[5]);

    vector<double> next_s_vals = computeMinimumJerk({start_pos_s, start_vel_s, 0.0},
                                                    {end_pos_s, end_vel_s, 0.0},
                                                    path_plan_seconds,
                                                    delta_t);
    vector<double> next_d_vals = computeMinimumJerk({start_pos_d, 0.0, 0.0},
                                                    {end_pos_d, 0.0, 0.0},
                                                    path_plan_seconds,
                                                    delta_t);

    vector<double> next_x_vals = {};
    vector<double> next_y_vals = {};
    for (int i = 0; i < next_s_vals.size(); i++) {
        vector<double> xy = wp.getXY(fmod(next_s_vals[i], wp.max_s),
                                     next_d_vals[i],
                                     map_waypoints_s,
                                     map_waypoints_x,
                                     map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
    jerk_result result;
    result.path_x = next_x_vals;
    result.path_y = next_y_vals;
    result.last_s = next_s_vals[next_s_vals.size() - 1];
    result.last_d = next_d_vals[next_d_vals.size() - 1];
    return result;
}
