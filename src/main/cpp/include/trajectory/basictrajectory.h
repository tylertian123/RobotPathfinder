#pragma once

#include "paths.h"
#include "trajectory/basicmoment.h"
#include "robotspecs.h"
#include "math/vec2d.h"
#include <memory>
#include <vector>

namespace rpf {
    class BasicTrajectory {
    public:


    protected:
        std::unique_ptr<Path> path;
        std::vector<BasicMoment> moments;
        std::vector<Vec2D> hvecs;
        std::vector<double> patht;
        std::vector<double> pathr;
        double init_facing;

        RobotSpecs specs;
    };
}
