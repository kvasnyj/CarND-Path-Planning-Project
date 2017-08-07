# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Waypoints
Class waypoints. Load waypoints from ../data/highway_map.csv and then interpolate it by Spline.
I also move into this class all methods with waypoints like ClosestWaypoint(), NextWaypoint(), getCarXY() and so on.
In addition I add helpers methods:
* lane2D -convert lane number into d position
* D2Lane - convert d position into lane number
