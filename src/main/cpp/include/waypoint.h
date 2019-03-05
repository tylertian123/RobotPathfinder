#pragma once

#include <limits>
#include "math/vec2d.h"

namespace rpf {
    struct Waypoint {
        Waypoint() {}
        Waypoint(const double &x, const double &y, const double &h) : x(x), y(y), heading(h) {}
        Waypoint(const Vec2D &v, double h) : x(v.x), y(v.y), heading(h) {}

        double x;
        double y;
        double heading;
        double velocity = std::numeric_limits<double>::quiet_NaN();

        operator Vec2D() const {
            return Vec2D(x, y);
        }
    };
}
