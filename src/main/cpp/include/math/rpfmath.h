#pragma once

namespace rpf {
    /* RobotPathfinder Math */

    // Linear Interpolation
    double lerp(double, double, double);
    // Restrict Angle
    double rangle(double);
    // Mirror Angle
    double mangle(double, double);
    // The constant pi
    constexpr double pi = 3.141592653589793238462643383279502884;
}
