#pragma once

#include "paths.h"
#include "waypoint.h"
#include <limits>
#include <vector>

namespace rpf {
    struct TrajectoryParams {
        std::vector<Waypoint> waypoints;
        double alpha = std::numeric_limits<double>::quiet_NaN();
        int sample_count;
        bool is_tank;
        PathType type;
    };
} // namespace rpf
