#pragma once

#include "paths.h"
#include "trajectory/basicmoment.h"
#include "robotspecs.h"
#include "trajectoryparams.h"
#include "math/vec2d.h"
#include <memory>
#include <vector>
#include <list>

namespace rpf {
    class BasicTrajectory {
    public:
        BasicTrajectory(const RobotSpecs &specs, const TrajectoryParams &params) : specs(specs), params(params) {
            path = std::shared_ptr<Path>(construct_path(params.waypoints, params.alpha, params.type));
            auto &waypoints = params.waypoints;

            if(params.is_tank) {
                path->set_base(specs.base_width / 2);
            }
            double ds = 1.0 / params.seg_count;
            double total = path->compute_len(params.seg_count + 1);
            double dpi = total / params.seg_count;
            
            std::vector<double> mv;
            std::list<std::pair<double, double>> constraints;

            double wpdt = 1.0 / (waypoints.size() - 1);
            for(int i = 1; i < waypoints.size() - 1; i ++) {
                if(!std::isnan(waypoints[i].velocity)) {
                    constraints.push_back(std::make_pair(path->t2s(i * wpdt) * total, waypoints[i].velocity));
                }
            }

            if(params.is_tank) {
                for(int i = 0; i < params.seg_count; i ++) {
                    double t = path->s2t(ds * i);
                    patht.push_back(t);
                    
                }
            }
        }

    protected:
        std::shared_ptr<Path> path = nullptr;
        std::vector<BasicMoment> moments;
        std::vector<Vec2D> hvecs;
        std::vector<double> patht;
        std::vector<double> pathr;
        double init_facing;

        RobotSpecs specs;
        TrajectoryParams params;
    };
}
