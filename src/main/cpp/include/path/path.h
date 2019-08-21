#pragma once

#include "segments.h"
#include "waypoint.h"
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace rpf {
    enum PathType : int {
        BEZIER = 1,
        CUBIC_HERMITE = 2,
        QUINTIC_HERMITE = 3,
    };

    class Path {
    public:
        Path(const std::vector<Waypoint> &waypoints, double alpha, PathType type);

        inline void set_base(double base_radius) {
            this->base_radius = base_radius;
        }
        inline double get_base() const {
            return base_radius;
        }

        Vec2D at(double) const;
        Vec2D deriv_at(double) const;
        Vec2D second_deriv_at(double) const;
        std::pair<Vec2D, Vec2D> wheels_at(double) const;

        double compute_len(int);

        inline double get_len() const {
            return total_len;
        }

        double s2t(double) const;
        double t2s(double) const;

        inline double get_alpha() const {
            return alpha;
        }
        inline PathType get_type() const {
            return type;
        }
        inline std::vector<Waypoint> &get_waypoints() {
            return waypoints;
        }
        inline const std::vector<Waypoint> &get_waypoints() const {
            return waypoints;
        }
        inline bool get_backwards() const {
            return backwards;
        }
        inline void set_backwards(bool backwards) {
            this->backwards = backwards;
        }

        std::shared_ptr<Path> mirror_fb() const;
        std::shared_ptr<Path> mirror_lr() const;
        std::shared_ptr<Path> retrace() const;

        void update(double, const Vec2D &, const Vec2D &, const Vec2D &);

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
} // namespace rpf
