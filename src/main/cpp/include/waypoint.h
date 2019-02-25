#pragma once

#include <limits>
#include "vec2d.h"

namespace rpf {
    struct Waypoint {
        double x;
        double y;
        double heading;
        double velocity = std::numeric_limits<double>::quiet_NaN();

        operator Vec2D() const {
            return Vec2D(x, y);
        }
    };
}
