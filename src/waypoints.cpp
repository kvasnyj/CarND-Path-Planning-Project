//
// Created by kvasnyj on 5/8/17.
//


#include <fstream>
#include <uWS/uWS.h>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "MathHelper.h"
#include "waypoints.h"
#include "spline.h"


using namespace std;

double waypoints::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int waypoints::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int waypoints::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > MathHelper::pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}


vector<double> waypoints::getCarXY(double car_x, double car_y, double theta, double wx, double wy)
{
    vector<double> results;

    // convert to local coordinates
    float deltax = (wx - car_x);
    float deltay = (wy - car_y);
    results.push_back(deltax*cos(theta) + deltay*sin(theta));
    results.push_back(-deltax*sin(theta) + deltay*cos(theta));
    return results;
}

vector<vector<double>> waypoints::getCarWPSegement(double car_x, double car_y, double car_yaw, double d, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy)
{
    vector<double> wpx;
    vector<double> wpy;
    vector<vector<double>> results;
    double theta = MathHelper::deg2rad(car_yaw);

    int closestWaypoint = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
    int previous = closestWaypoint - 6;
    if (previous < 0) {
        previous += maps_x.size();
    }
    cout << "waypoints: ";
    for (int i = 0; i < 25; i++) {
        int next = (previous+i)%maps_x.size();
        vector<double> localxy = getCarXY(car_x, car_y, theta, (maps_x[next]+d*maps_dx[next]), (maps_y[next]+d*maps_dy[next]));
        cout << next << ":" << localxy[0] << ":" << localxy[1] << ",";
        wpx.push_back(localxy[0]);
        wpy.push_back(localxy[1]);
    }
    cout << endl;
    results.push_back(wpx);
    results.push_back(wpy);

    return results;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> waypoints::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> waypoints::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-MathHelper::pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}


double waypoints::lane2D(int lane) {
    if (lane == 1) return 2.0;
    if (lane == 2) return 6.0;
    if (lane == 3) return 10.0;

    return 0;
}

int waypoints::D2Lane(double d) {
    if (d < 4) return 1;
    if (d >= 4.0 & d < 8.0) return 2;
    return 3;
}

waypoints::waypoints()
{
    max_s = 6945.554;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors


    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);

    }

    vector<double> waypoint_spline_t = {};
    int map_waypoints_size = map_waypoints_x.size();
    for (int i=0; i<map_waypoints_size; i++)
    {
        double t = (double)i / (double)map_waypoints_size;
        waypoint_spline_t.push_back(t);
    }

    tk::spline waypoint_spline_x;
    waypoint_spline_x.set_points(waypoint_spline_t, map_waypoints_x);
    tk::spline waypoint_spline_y;
    waypoint_spline_y.set_points(waypoint_spline_t, map_waypoints_y);
    tk::spline waypoint_spline_s;
    waypoint_spline_s.set_points(waypoint_spline_t, map_waypoints_s);
    tk::spline waypoint_spline_dx;
    waypoint_spline_dx.set_points(waypoint_spline_t, map_waypoints_dx);
    tk::spline waypoint_spline_dy;
    waypoint_spline_dy.set_points(waypoint_spline_t, map_waypoints_dy);

    vector<double> map_waypoints_x_new;
    vector<double> map_waypoints_y_new;
    vector<double> map_waypoints_s_new;
    vector<double> map_waypoints_dx_new;
    vector<double> map_waypoints_dy_new;

    for (int i=0; i<10000; i++)
    {
        double t = (double)i / (double)10000;
        map_waypoints_x_new.push_back(waypoint_spline_x(t));
        map_waypoints_y_new.push_back(waypoint_spline_y(t));
        map_waypoints_s_new.push_back(waypoint_spline_s(t));
        map_waypoints_dx_new.push_back(waypoint_spline_dx(t));
        map_waypoints_dy_new.push_back(waypoint_spline_dy(t));
    }

    map_waypoints_x  = map_waypoints_x_new;
    map_waypoints_y  = map_waypoints_y_new;
    map_waypoints_s  = map_waypoints_s_new;
    map_waypoints_dx = map_waypoints_dx_new;
    map_waypoints_dy = map_waypoints_dy_new;
}