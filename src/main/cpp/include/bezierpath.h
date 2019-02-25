#pragma once

#include "beziersegment.h"
#include "path.h"
#include "vec2d.h"
#include <stdexcept>

namespace rpf {
    class BezierPath : public Path {
    public:
        BezierPath(const std::vector<Waypoint> &waypoints, double alpha) : waypoints(waypoints), alpha(alpha) {
            type = PathType::BEZIER;

            if(waypoints.size() < 2) {
                throw std::runtime_error("Not enough waypoints");
            }

            segments.resize(waypoints.size() - 1);
            
            for(int i = 0; i < waypoints.size() - 1; i ++) {
                segments.push_back(BezierSegment::from_hermite((Vec2D) waypoints[i], (Vec2D) waypoints[i + 1],
                        Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                        Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)));
            }
        }
    };
}
