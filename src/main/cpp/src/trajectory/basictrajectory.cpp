#include "trajectory/basictrajectory.h"

namespace rpf {
    std::shared_ptr<Path> BasicTrajectory::get_path() {
        return path;
    }
    std::shared_ptr<const Path> BasicTrajectory::get_path() const {
        return path;
    }
    std::vector<BasicMoment>& BasicTrajectory::get_moments() {
        return moments;
    }
    const std::vector<BasicMoment>& BasicTrajectory::get_moments() const {
        return moments;
    }
    double BasicTrajectory::get_init_facing() const {
        return init_facing;
    }

    RobotSpecs& BasicTrajectory::get_specs() {
        return specs;
    }
    const RobotSpecs& BasicTrajectory::get_specs() const {
        return specs;
    }
    TrajectoryParams& BasicTrajectory::get_params() {
        return params;
    }
    const TrajectoryParams& BasicTrajectory::get_params() const {
        return params;
    }
}
