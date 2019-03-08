#pragma once

#include "vec2d.h"

namespace rpf {
    /* RobotPathfinder Math */

    // Linear Interpolation
    double lerp(double, double, double);
    // Restrict Angle
    double rangle(double);
    // Mirror Angle
    double mangle(double, double);
    // Linearly interpolates between angles
    double langle(double, double, double);
    double langle(Vec2D, Vec2D, double);
    // Computes curvature
    double curvature(double, double, double, double);
    // The constant pi
    constexpr double pi = 3.141592653589793238462643383279502884;
}
