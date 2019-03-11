#pragma once

#include "waypoint.h"
#include "segments.h"
#include <memory>
#include <vector>
#include <limits>
#include <utility>
#include <cmath>

namespace rpf {
    enum PathType : int {
        BEZIER = 1,
        CUBIC_HERMITE = 2,
        QUINTIC_HERMITE = 3,
    };

    class Path {
    public:
        Path(const std::vector<Waypoint> &waypoints, double alpha, PathType type) : 
                waypoints(waypoints), alpha(alpha), type(type) {

            if(waypoints.size() < 2) {
                throw std::invalid_argument("Not enough waypoints");
            }
            segments.reserve(waypoints.size() - 1);
            switch(type) {
            case PathType::BEZIER:
                for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                    segments.push_back(std::make_unique<BezierSegment>(BezierSegment::from_hermite(
                        static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                        Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                        Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)
                    )));
                }
                break;
            case PathType::CUBIC_HERMITE:
                for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                    segments.push_back(std::make_unique<CubicSegment>(
                        static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                        Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                        Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)
                    ));
                }
                break;
            case PathType::QUINTIC_HERMITE:
                for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                    segments.push_back(std::make_unique<QuinticSegment>(
                        static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                        Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                        Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha),
                        Vec2D(0, 0), Vec2D(0, 0)
                    ));
                }
                break;
            }
        }

        // TODO: INLINE ALL THESE GETTERS!
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
        std::vector<Waypoint>& get_waypoints();
        const std::vector<Waypoint>& get_waypoints() const;
        bool get_backwards() const;
        void set_backwards(bool);

        std::shared_ptr<Path> mirror_fb() const;
        std::shared_ptr<Path> mirror_lr() const;
        std::shared_ptr<Path> retrace() const;

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
}
