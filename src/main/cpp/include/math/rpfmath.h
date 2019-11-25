#pragma once

#include "vec2d.h"

namespace rpf {
    /* RobotPathfinder Math */

    // Linear Interpolation
    double lerp(double a, double b, double f);
    // Restrict Angle
    double restrict_angle(double angle);
    // Mirror Angle
    double mirror_angle(double angle, double ref);
    // Linearly interpolates between angles
    double lerp_angle(double a, double b, double f);
    double lerp_angle(Vec2D a, Vec2D b, double f);
    // Restrict absolute value
    double restrict_abs(double x, double m);
    // Computes curvature
    double curvature(double dx, double ddx, double dy, double ddy);
    // The constant pi
    constexpr double pi = 3.141592653589793238462643383279502884;
} // namespace rpf
