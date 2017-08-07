# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Waypoints
Class waypoints. Load waypoints from ../data/highway_map.csv and then interpolate it by Spline.
I also move into this class all methods with waypoints like ClosestWaypoint(), NextWaypoint(), getCarXY() and so on.
Also, I add helpers methods:
* lane2D -convert lane number into d position
* D2Lane - convert d position into lane number

### Planner
All logic regarding creates new waypoints I realized in separate planner class with the following methods:
* Constructor - init constants
* FollowingWP - basic method, generate new waypoints 
* fill_other_cars - parse sensor fusion data
* distanceToClosestCar - distance to the car in front or rear direction. If there are no cars, return 100 meters (not 6945.554 to balance cost function)
* costOfLaneChange and  costOfStraightCourse - cost of maneuver. 
* lowestCostAction - take action with the lowest cost.
* determineNewCourseSetpoints and determineNewStraightCourseSetpoints - define new coordinates and speed after maneuver. In the case of changing lane finale speed slightly less that start speed. In the case of going straight final speed dependence from the speed of the car ahead. 
* computeMinimumJerkMapPath - generate path with minimum jerk 

### Further improvements 
* Prediction of lane changing for other cars. Now I predict the only position for going straights cars. 
* Better algorithm for "following car" case.  
