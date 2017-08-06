//
// Created by kvasnyj on 5/8/17.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H
#include <vector>

using namespace std;

class waypoints {
public:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // The max s value before wrapping around the track back to 0
    double max_s;

    waypoints();

    double distance(double x1, double y1, double x2, double y2);
    double lane2D(int lane);
    int D2Lane(double d);
    int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getCarXY(double car_x, double car_y, double theta, double wx, double wy);
    vector<vector<double>> getCarWPSegement(double car_x, double car_y, double car_yaw, double d, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy);
    vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
};


#endif //PATH_PLANNING_WAYPOINTS_H
