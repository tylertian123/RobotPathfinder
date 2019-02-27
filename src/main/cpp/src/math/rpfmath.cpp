#include "math/rpfmath.h"

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
}
