#include "trajectory/tankdrivetrajectory.h"

namespace rpf {

    TankDriveTrajectory::TankDriveTrajectory(const BasicTrajectory &traj)
            : path(traj.path), patht(traj.patht), specs(traj.specs), params(traj.params),
              init_facing(traj.init_facing) {
        if (!params.is_tank) {
            throw std::invalid_argument("Base trajectory must be tank");
        }

        path->set_base(specs.base_width / 2);
        moments.reserve(traj.moments.size());
        // Initialize first moment
        if (!std::isnan(params.waypoints[0].velocity)) {
            double v = traj.moments[0].vel;
            double d = v / (*traj.pathr)[0] * specs.base_width / 2;
            // Apply the velocity formula (derived below) to find the wheel velocities for the two
            // wheels
            moments.push_back(
                    TankDriveMoment(0, 0, v - d, v + d, 0, 0, traj.moments[0].heading, 0));
        }
        else {
            moments.push_back(TankDriveMoment(0, 0, 0, 0, 0, 0, traj.moments[0].heading, 0));
        }

        // Use numerical integration for each moment to figure out the values
        // This variable keeps track of where the wheels were in the last iteration.
        auto init = path->wheels_at(0);
        moments[0].init_facing = traj.init_facing;
        for (size_t i = 1; i < traj.moments.size(); i++) {
            // First find where the wheels are at this moment and integrate the length
            auto wheels = path->wheels_at((*traj.patht)[i]);
            double dl = init.first.dist(wheels.first);
            double dr = init.second.dist(wheels.second);
            double dt = traj.moments[i].time - traj.moments[i - 1].time;

            // Find out the velocity of the two wheels
            /*
             * The formula for velocity is derived as follows:
             * Start with the equation:
             * 1. w = v/r, where w is the angular velocity, r is the radius of the path, and v is
             * the velocity
             *
             * 1. From the equation we can get v = wr
             * 2. Let b represent the base radius; then, the radius for the left wheel is r - b, the
             * radius for the right wheel is r + b
             * 3. Substitute r: v1 = w(r - b), v2 = w(r + b), where v1 is the left wheel velocity,
             * v2 is the right wheel velocity
             * 4. Substitute w: v1 = (v/r) (r - b), v2 = (v/r) (r + b)
             * 5. Distribute: v1 = v - (v/r) * b, v2 = v + (v/r) * b
             *
             * The nice thing about using path radius to figure out the velocity is now we can have
             * negative velocities when the turn is too tight and the wheel has to move backwards,
             * unlike the distance difference which is always positive.
             */
            init = wheels;
            double d = traj.moments[i].vel / (*traj.pathr)[i] * (specs.base_width / 2);
            double lv = traj.moments[i].vel - d;
            double rv = traj.moments[i].vel + d;

            // If the corresponding wheel velocity is negative, then the distance difference must
            // also be negative
            if (lv < 0) {
                dl = -dl;
            }
            if (rv < 0) {
                dr = -dr;
            }

            // Create a new moment and set the acceleration of the last moment
            moments.push_back(TankDriveMoment(moments[i - 1].l_pos + dl, moments[i - 1].r_pos + dr,
                    lv, rv, 0, 0, traj.moments[i].heading, traj.moments[i].time, traj.init_facing));
            moments[i - 1].l_accel = (lv - moments[i - 1].l_vel) / dt;
            moments[i - 1].r_accel = (rv - moments[i - 1].r_vel) / dt;
        }
    }

    std::pair<std::size_t, std::size_t> TankDriveTrajectory::search_moments(double t) const {
        std::size_t start = 0;
        std::size_t end = moments.size() - 1;
        std::size_t mid;

        // Time out of range - take the last moment
        if (t >= total_time()) {
            return std::make_pair(moments.size() - 1, moments.size() - 1);
        }

        while (true) {
            mid = (start + end) / 2;
            double mid_time = moments[mid].time;
            // Exact match
            if (mid_time == t || mid == moments.size() - 1) {
                return std::make_pair(mid, mid);
            }
            // Time is sandwiched between two moments
            double next_time = moments[mid + 1].time;
            if (mid_time <= t && next_time >= t) {
                return std::make_pair(mid, mid + 1);
            }
            // Time out of range - take the first moment
            if (mid == 0) {
                return std::make_pair(0, 0);
            }
            if (mid_time < t) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }

    TankDriveMoment TankDriveTrajectory::get(double t) const {
        auto m = search_moments(t);
        // Exact match - return it
        if (m.first == m.second) {
            return moments[m.first];
        }
        else {
            // Otherwise linearly interpolate
            double f =
                    (t - moments[m.first].time) / (moments[m.second].time - moments[m.first].time);
            auto &current = moments[m.first];
            auto &next = moments[m.second];

            TankDriveMoment moment(rpf::lerp(current.l_pos, next.l_pos, f),
                    rpf::lerp(current.r_pos, next.r_pos, f),
                    rpf::lerp(current.l_vel, next.l_vel, f),
                    rpf::lerp(current.r_vel, next.r_vel, f),
                    rpf::lerp(current.l_accel, next.l_accel, f),
                    rpf::lerp(current.r_accel, next.r_accel, f),
                    rpf::lerp_angle(current.heading, next.heading, f), t, init_facing);
            moment.backwards = backwards;
            return moment;
        }
    }

    Waypoint TankDriveTrajectory::get_pos(double t) const {
        auto m = search_moments(t);
        // Calculate path time using lookup table
        double pt;
        if (m.first == m.second) {
            // Exact match
            pt = (*patht)[m.first];
        }
        else {
            // Otherwise linearly interpolate
            double t1 = (*patht)[m.first];
            double t2 = (*patht)[m.second];
            double f =
                    (t - moments[m.first].time) / (moments[m.second].time - moments[m.first].time);
            pt = lerp(t1, t2, f);
        }

        auto pos = path->at(pt);
        auto deriv = path->deriv_at(pt);
        // From the derivative calculate the heading
        return Waypoint(pos, std::atan2(deriv.y, deriv.x));
    }

    std::shared_ptr<TankDriveTrajectory> TankDriveTrajectory::mirror_lr() const {
        auto p = path->mirror_lr();
        double ref = params.waypoints[0].heading;

        std::vector<TankDriveMoment> m;
        m.reserve(moments.size());
        for (const auto &moment : moments) {
            TankDriveMoment nm(moment.r_pos, moment.l_pos, moment.r_vel, moment.l_vel,
                    moment.r_accel, moment.l_accel, rpf::mirror_angle(moment.heading, ref), moment.time,
                    moment.init_facing);
            nm.backwards = backwards;
            m.push_back(nm);
        }

        return std::shared_ptr<TankDriveTrajectory>(
                new TankDriveTrajectory(p, std::move(m), backwards, specs, params));
    }
    std::shared_ptr<TankDriveTrajectory> TankDriveTrajectory::mirror_fb() const {
        auto p = path->mirror_fb();
        double ref = rpf::restrict_angle(params.waypoints[0].heading + rpf::pi / 2);

        std::vector<TankDriveMoment> m;
        m.reserve(moments.size());
        for (const auto &moment : moments) {
            TankDriveMoment nm(-moment.l_pos, -moment.r_pos, -moment.l_vel, -moment.r_vel,
                    -moment.l_accel, -moment.r_accel, rpf::mirror_angle(moment.heading, ref), moment.time,
                    moment.init_facing);
            nm.backwards = !backwards;
            m.push_back(nm);
        }
        return std::shared_ptr<TankDriveTrajectory>(
                new TankDriveTrajectory(p, std::move(m), !backwards, specs, params));
    }
    std::shared_ptr<TankDriveTrajectory> TankDriveTrajectory::retrace() const {
        auto p = path->retrace();

        std::vector<TankDriveMoment> m;
        m.reserve(moments.size());
        auto &last = moments[moments.size() - 1];
        for (auto rit = moments.rbegin(); rit != moments.rend(); ++rit) {
            const auto &moment = *rit;
            /*
             * To generate the new moments, first the order of the moments has to be reversed, since
             * we are now starting from the end. The first moments should have less distance than
             * the later moments, so when iterating backwards, the position of the moment is
             * subtracted from the total distance, then negated since we're driving backwards.
             * Velocity is also negated, but since it's not accumulative, it does not need to be
             * subtracted from the total. Finally, acceleration is negated once for driving
             * backwards, and negated again because the direction of time is reversed, and together
             * they cancel out, resulting in no change. The heading is flipped 180 degrees, and the
             * time is subtracted from the total.
             */
            TankDriveMoment nm(-(last.l_pos - moment.l_pos), -(last.r_pos - moment.r_pos),
                    -moment.l_vel, -moment.r_vel, moment.l_accel, moment.r_accel, -moment.heading,
                    last.time - moment.time, params.waypoints[params.waypoints.size() - 1].heading);
            nm.backwards = !backwards;
            m.push_back(nm);
        }
        return std::shared_ptr<TankDriveTrajectory>(
                new TankDriveTrajectory(p, std::move(m), !backwards, specs, params));
    }
} // namespace rpf
