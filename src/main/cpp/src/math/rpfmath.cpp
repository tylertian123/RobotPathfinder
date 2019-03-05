#include "math/rpfmath.h"

#include <cmath>

namespace rpf {
    double lerp(double a, double b, double f) {
        return (a * (1.0 - f)) + (b * f);
    }

    double rangle(double angle) {
        if(angle <= pi && angle > -pi) {
            return angle;
        }
        while(angle > pi) {
            angle -= pi;
            angle -= pi;
        }
        while(angle <= -pi) {
            angle += pi;
            angle += pi;
        }
        return angle;
    }

    double mangle(double angle, double ref) {
        return rangle(angle - 2 * (angle - ref));
    }

    double curvature(double dx, double ddx, double dy, double ddy) {
        return (dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 3.0 / 2.0);
    }
}
