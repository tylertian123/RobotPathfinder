#pragma once

#include "beziersegment.h"
#include "path.h"
#include "vec2d.h"
#include <stdexcept>
#include <cmath>

namespace rpf {
    class BezierPath : public rpf::Path {
    public:
        BezierPath(const std::vector<Waypoint> &waypoints, double alpha) {
            this->waypoints = waypoints;
            this->alpha = alpha;
            type = PathType::BEZIER;

            if(waypoints.size() < 2) {
                throw std::runtime_error("Not enough waypoints");
            }

            segments.resize(waypoints.size() - 1);
            
            for(int i = 0; i < waypoints.size() - 1; i ++) {
                segments.push_back(std::make_unique<BezierSegment>(BezierSegment::from_hermite(
                    static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                    Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                    Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)
                )));
            }
        }
    };
}
