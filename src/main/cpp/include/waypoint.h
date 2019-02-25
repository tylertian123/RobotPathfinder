#pragma once

#include <limits>

namespace rpf {
    struct Waypoint {
        double x;
        double y;
        double heading;
        double velocity = std::numeric_limits<double>::quiet_NaN();
    };
}
