#pragma once

#include "paths.h"
#include "trajectory/basicmoment.h"
#include "robotspecs.h"
#include "trajectoryparams.h"
#include "math/vec2d.h"
#include <memory>
#include <vector>
#include <list>
#include <unordered_set>
#include <limits>
#include <stdexcept>

namespace rpf {

    class BasicTrajectory {
    public:
        BasicTrajectory(const RobotSpecs &specs, const TrajectoryParams &params);

        inline std::shared_ptr<Path> get_path() {
            return path;
        }
        inline std::shared_ptr<const Path> get_path() const {
            return path;
        }
        inline std::vector<BasicMoment>& get_moments() {
            return moments;
        }
        inline const std::vector<BasicMoment>& get_moments() const {
            return moments;
        }
        inline double get_init_facing() const {
            return init_facing;
        }

        inline RobotSpecs& get_specs() {
            return specs;
        }
        inline const RobotSpecs& get_specs() const {
            return specs;
        }
        inline TrajectoryParams& get_params() {
            return params;
        }
        inline const TrajectoryParams& get_params() const {
            return params;
        }

        inline double total_time() const {
            return moments[moments.size() - 1].time;
        }
        inline bool is_tank() const {
            return params.is_tank;
        }

        BasicMoment get(double) const;

        std::shared_ptr<BasicTrajectory> mirror_lr() const;
        std::shared_ptr<BasicTrajectory> mirror_fb() const;
        std::shared_ptr<BasicTrajectory> retrace() const;

        friend class TankDriveTrajectory;

    protected:
        
        BasicTrajectory(std::shared_ptr<Path> path, std::vector<BasicMoment> &&moments, bool backwards, 
                const RobotSpecs &specs, const TrajectoryParams &params) 
                : path(path), moments(moments), backwards(backwards), specs(specs), params(params), init_facing(moments[0].init_facing) {}


        std::shared_ptr<Path> path = nullptr;
        std::vector<BasicMoment> moments;

        bool backwards = false;

        RobotSpecs specs;
        TrajectoryParams params;

        double init_facing;

        std::vector<double> patht;
        std::vector<double> pathr;
    };
}
