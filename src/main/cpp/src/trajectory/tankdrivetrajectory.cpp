#include "trajectory/tankdrivetrajectory.h"

namespace rpf {

    TankDriveMoment TankDriveTrajectory::get(double time) const {
        size_t start = 0;
        size_t end = moments.size() - 1;
        size_t mid;

        if(time >= total_time()) {
            return moments[moments.size() - 1];
        }

        while(true) {
            mid = (start + end) / 2;
            double mid_time = moments[mid].time;

            if(mid_time == time || mid == moments.size() - 1) {
                return moments[mid];
            }
            
            double next_time = moments[mid + 1].time;
            if(mid_time <= time && next_time >= time) {
                double f = (time - mid_time) / (next_time - mid_time);
                auto &current = moments[mid];
                auto &next = moments[mid + 1];

                TankDriveMoment m(rpf::lerp(current.l_dist, next.l_dist, f), rpf::lerp(current.r_dist, next.r_dist, f),
                        rpf::lerp(current.l_vel, next.l_vel, f), rpf::lerp(current.r_vel, next.r_vel, f),
                        rpf::lerp(current.l_accel, next.l_accel, f), rpf::lerp(current.r_accel, next.r_accel, f),
                        rpf::langle(current.heading, next.heading, f), time, init_facing);
                m.backwards = backwards;
                return m;
            }
            if(mid == 0) {
                return moments[mid];
            }
            if(mid_time < time) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }
    
    std::shared_ptr<TankDriveTrajectory> TankDriveTrajectory::mirror_lr() const {
        auto p = path->mirror_lr();
        double ref = params.waypoints[0].heading;

        std::vector<TankDriveMoment> m;
        m.reserve(moments.size());
        for(auto moment : moments) {
            TankDriveMoment nm(moment.r_dist, moment.l_dist, moment.r_vel, moment.l_vel, moment.r_accel, moment.l_accel, 
                    rpf::mangle(moment.heading, ref), moment.time, moment.init_facing);
            nm.backwards = backwards;
            m.push_back(nm);
        }

        return std::shared_ptr<TankDriveTrajectory>(new TankDriveTrajectory(p, std::move(m), backwards, specs, params));
    }
}
