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

    double BasicTrajectory::total_time() const {
        return moments[moments.size() - 1].time;
    }
    bool BasicTrajectory::is_tank() const {
        return params.is_tank;
    }

    BasicMoment BasicTrajectory::get(double time) const {
        size_t start = 0;
        size_t end = moments.size() - 1;
        size_t mid;

        if(time >= total_time()) {
            return moments[moments.size() - 1];
        }

        while(true) {
            mid = (start + end) / 2;
            double mid_time = moments[mid].time;

            if(mid_time == time || mid == moments.size() - 1 || mid == 0) {
                return moments[mid];
            }
            
            double next_time = moments[mid + 1].time;
            if(mid_time <= time && next_time >= time) {
                double f = (time - mid_time) / (next_time - mid_time);
                auto &current = moments[mid];
                auto &next = moments[mid + 1];
                BasicMoment m(rpf::lerp(current.dist, next.dist, f), rpf::lerp(current.vel, next.vel, f),
                        rpf::lerp(current.accel, next.accel, f), rpf::langle(current.heading, next.heading, f), time, init_facing);
                m.backwards = backwards;
                return m;
            }
            if(mid_time < time) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }

    std::shared_ptr<BasicTrajectory> BasicTrajectory::mirror_lr() const {
        auto p = path->mirror_lr();
        double ref = params.waypoints[0].heading;
        
        std::vector<BasicMoment> m;
        m.reserve(moments.size());

        for(size_t i = 0; i < moments.size(); i ++) {
            BasicMoment moment(moments[i]);
            moment.heading = rpf::mangle(moment.heading, ref);
            moment.init_facing = params.waypoints[0].heading;
            m.push_back(moment);
        }
        return std::shared_ptr<BasicTrajectory>(new BasicTrajectory(p, std::move(m), backwards, specs, params));
    }
    std::shared_ptr<BasicTrajectory> BasicTrajectory::mirror_fb() const {
        auto p = path->mirror_fb();
        double ref = params.waypoints[0].heading + rpf::pi / 2;

        std::vector<BasicMoment> m;
        m.reserve(moments.size());
        for(size_t i = 0; i < moments.size(); i ++) {
            BasicMoment moment(-moments[i].dist, -moments[i].vel, moments[i].accel, 
                    rpf::mangle(moments[i].heading, ref), moments[i].time);
            moment.init_facing = params.waypoints[0].heading;
            moment.backwards = true;
            m.push_back(moment);
        }

        return std::shared_ptr<BasicTrajectory>(new BasicTrajectory(p, std::move(m), !backwards, specs, params));
    }
    std::shared_ptr<BasicTrajectory> BasicTrajectory::retrace() const {
        auto p = path->retrace();
        
        std::vector<BasicMoment> m;
        m.reserve(moments.size());
        auto &last = moments[moments.size() - 1];
        for(size_t i = 0; i < moments.size(); i ++) {
            auto &current = moments[moments.size() - 1 - i];

            BasicMoment moment(-(last.dist - current.dist), -current.vel, current.accel, -current.heading, last.time - current.time);
            moment.init_facing = params.waypoints[params.waypoints.size() - 1].heading;
            moment.backwards = true;
            m.push_back(moment);
        }

        return std::shared_ptr<BasicTrajectory>(new BasicTrajectory(p, std::move(m), !backwards, specs, params));
    }
}
