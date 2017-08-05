//
// Created by kvasnyj on 5/8/17.
//

#ifndef PATH_PLANNING_MATHHELPER_H
#define PATH_PLANNING_MATHHELPER_H
#include <math.h>

class MathHelper {
public:
    static constexpr double pi() { return M_PI; }
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }

};


#endif //PATH_PLANNING_MATHHELPER_H
