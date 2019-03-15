#pragma once

#include "waypoint.h"
#include "paths.h"
#include <vector>
#include <limits>

namespace rpf {
    struct TrajectoryParams {
        std::vector<Waypoint> waypoints;
        double alpha = std::numeric_limits<double>::quiet_NaN();
        int sample_count;
        bool is_tank;
        PathType type;
    };
}
