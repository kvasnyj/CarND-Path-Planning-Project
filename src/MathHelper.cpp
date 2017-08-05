//
// Created by kvasnyj on 5/8/17.
//

#include "MathHelper.h"
#include <math.h>

// For converting back and forth between radians and degrees.
static constexpr double pi() { return M_PI; }

static double deg2rad(double x) { return x * pi() / 180; }

static double rad2deg(double x) { return x * 180 / pi(); }