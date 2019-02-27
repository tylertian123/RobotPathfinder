#pragma once

#include "waypoint.h"
#include "segment/splinesegment.h"
#include <memory>
#include <vector>
#include <limits>
#include <utility>

namespace rpf {
    enum PathType : int {
        BEZIER = 1,
        CUBIC_HERMITE = 2,
        QUINTIC_HERMITE = 3,
    };

    class Path {
    public:
        virtual ~Path() = 0;

        void set_base(double);
        double get_base() const;

        Vec2D at(double) const;
        Vec2D deriv_at(double) const;
        Vec2D second_deriv_at(double) const;
        std::pair<Vec2D, Vec2D> wheels_at(double) const;
        
        double compute_len(int);
        double get_len() const;

        double s2t(double) const;
        double t2s(double) const;

        double get_alpha() const;
        PathType get_type() const;
        std::vector<Waypoint> get_waypoints() const;
        bool get_backwards() const;
        void set_backwards(bool);

        Path* mirror_fb() const;
        Path* mirror_lr() const;
        Path* retrace() const;

    protected:
        std::vector<Waypoint> waypoints;
        double alpha;
        std::vector<std::unique_ptr<SplineSegment>> segments;
        PathType type;
        
        double total_len = std::numeric_limits<double>::quiet_NaN();
        std::vector<std::pair<double, double>> s2t_table;
        
        bool backwards = false;
        double base_radius;
    };

    Path* construct_path(const std::vector<Waypoint> &, double, PathType);
}
