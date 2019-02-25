#pragma once

#include "waypoint.h"
#include "splinesegment.h"
#include <memory>
#include <vector>
#include <limits>
#include <utility>

namespace rpf {
    enum PathType {
        BEZIER = 1,
        CUBIC_HERMITE = 2,
        QUINTIC_HERMITE = 3,
    };

    class Path {
    public:

    protected:
        Path() {}

        std::vector<Waypoint> waypoints;
        std::vector<std::unique_ptr<SplineSegment>> segments;
        PathType type;
        
        double total_len = std::numeric_limits<double>::quiet_NaN();
        std::vector<std::pair<double, double>> s2t_table;
        
        bool backwards = false;
        double base_radius;
    };
}
