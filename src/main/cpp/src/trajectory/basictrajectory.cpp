#include "trajectory/basictrajectory.h"

namespace rpf {

    /*
     * Abandon all hope, ye who enter here.
     */

    /*
     * The algorithm used to generate these trajectories are based almost entirely on the algorithm
     * from Team 254 The Cheesy Poofs. Video here: https://youtu.be/8319J1BEHwM
     */

    BasicTrajectory::BasicTrajectory(const RobotSpecs &specs, const TrajectoryParams &params)
            : specs(specs), params(params) {
        // Make the path
        path = std::make_shared<Path>(params.waypoints, params.alpha, params.type);
        auto &waypoints = params.waypoints;

        if (params.is_tank) {
            path->set_base(specs.base_width / 2);
        }

        /*
         * Because most parametric polynomials don't have constant speed (i.e. the magnitude of the
         * derivative is non-constant), we use some special processing to make samples the same
         * physical distance apart. Instead of getting positions from the path and iterating the
         * time, we calculate the whole length of the path, and make each sample a constant length
         * away. The time value can then be found by calling the s2T method in Path, and any special
         * processing can be done with that.
         */
        // Instead of iterating over t, we iterate over s, which represents the fraction of the
        // total distance ds is the difference in the fraction of the total path length travelled
        // for each iteration
        double ds = 1.0 / (params.sample_count - 1);
        double total = path->compute_len(params.sample_count);
        // dpi stands for Distance Per Iteration, it is the distance travelled along the path for
        // each iteration
        double dpi = total / (params.sample_count - 1);

        // Extract and organize all the additional velocity constraints from the waypoints
        // The first element of each Pair of doubles holds the path distance for the constraint
        // The second element holds the velocity
        // Use a list because random access is never needed
        std::list<std::pair<double, double>> constraints;
        // Since waypoints are spaced evenly though time we can calculate the constant difference
        // here
        double wpdt = 1.0 / (waypoints.size() - 1);
        for (size_t i = 1; i < waypoints.size() - 1; i++) {
            if (!std::isnan(waypoints[i].velocity)) {
                if (std::abs(waypoints[i].velocity) > specs.max_v) {
                    throw std::invalid_argument(
                            "Waypoint velocity constraint is greater than the max velocity");
                }
                // Use t2S to find the fractional distance, then multiply by the total distance
                constraints.push_back(
                        std::make_pair(path->t2s(i * wpdt) * total, waypoints[i].velocity));
            }
        }

        // This array stores the theoretical max velocity at each point in this trajectory
        // This is needed for tank drive, since the robot has to slow down when turning
        // For regular basic trajectories every element of this array is set to the max velocity
        std::vector<double> mv;
        mv.reserve(params.sample_count);
        // This array stores the direction of the robot at each moment
        // Directions are generated in a separate process as the velocities and accelerations
        std::vector<double> headings;
        headings.reserve(params.sample_count);
        // patht and pathr are accessed by the TankDriveTrajectory constructor later
        // patht is also used to find the position given a time later
        // They store the t and radius of each of the sample points along the path
        patht->reserve(params.sample_count);
        if (params.is_tank) {
            // Note that pathr is not initialized
            pathr = std::make_shared<std::vector<double>>();
            pathr->reserve(params.sample_count);
        }
        /*
         * "Moments" represent a moment in time.
         * Each moment has a position, velocity, acceleration and time. The trajectory is made of a
         * collection of these generated Moments. Using them, at any given time we can (roughly, but
         * closely enough) determine the position, velocity and acceleration the robot is supposed
         * to be at.
         */
        moments.reserve(params.sample_count);

        if (params.is_tank) {
            // Tank drive trajectories require extra processing as described above
            for (int i = 0; i < params.sample_count; i++) {
                // Call s2T to translate between length and time
                double t = path->s2t(ds * i);
                // Store a value into patht for use by TankDriveTrajectory later
                patht->push_back(t);

                auto d = path->deriv_at(t);
                auto dd = path->second_deriv_at(t);
                // Use the curvature formula in multivariable calculus to figure out the curvature
                // at this point of the path
                double curvature = rpf::curvature(d.x, dd.x, d.y, dd.y);
                // The heading is generated as a by-product
                headings.push_back(std::atan2(d.y, d.x));
                // Store a value into pathr for use by TankDriveTrajectory later
                pathr->push_back(1 / curvature);
                /*
                 * The maximum speed for the entire robot is computed with a formula. Derivation
                 * here: Start with the equations:
                 * 1. (r - l) / b = w, where l and r are the wheel velocities, b is the base width
                 * and w (omega) is the angular velocity.
                 * 2. (l + r) / 2 = V, where l and r are the wheel velocities, and V is the overall
                 * velocity
                 * 3. w = V / R, where w is the angular velocity, V is the overall velocity, and R
                 * is the radius of the path.
                 *
                 * 1. Rearrange equation 1: wb = r - l, l = r - wb
                 * 2. Since we want the robot to go as fast as possible, the faster wheel has
                 * velocity Vmax
                 * 3. Assuming the right side is faster, r = Vmax, and by 1, l = Vmax - wb
                 * 4. Equation 2 becomes: (2Vmax - wb) / 2 = V
                 * 5. Substitute in equation 3, (2Vmax - (V / R)b) / 2 = V
                 * 6. Now solve for V: 2Vmax - (V / R)b = 2V, 2V + (V / R)b = 2Vmax, V(2 + b / R) =
                 * 2Vmax, V = 2Vmax / (2 + b / R), V = Vmax / (1 + b / (2R))
                 */
                mv.push_back(specs.max_v / (1 + specs.base_width / (2 * std::abs((*pathr)[i]))));
            }
        }
        else {
            // If the trajectory is just a basic trajectory, there's no need to slow down, so every
            // point's max velocity is the specified max velocity.
            for (int i = 0; i < params.sample_count; i++) {
                mv.push_back(specs.max_v);

                double t = path->s2t(ds * i);
                Vec2D d = path->deriv_at(t);
                // Even if the trajectory is not for tank drive robots, the heading still needs to
                // be calculated
                headings.push_back(std::atan2(d.y, d.x));
            }
        }

        /*
         * This array holds the difference in time between two moments.
         * During the forward and backwards passes, the time difference can be computed just using
         * simple division. If computed at the end, they would require more expensive calls to
         * sqrt().
         */
        std::vector<double> time_diff(
                params.sample_count - 1, std::numeric_limits<double>::quiet_NaN());
        // This is a set that stores all the indices of the moments of which their velocities cannot
        // be changed (as specified by the Waypoints)
        std::unordered_set<int> constrained;

        // Initialize the first moment of the array
        // If the velocity is specified then follow the constraints
        if (!std::isnan(waypoints[0].velocity)) {
            moments.push_back(BasicMoment(0, waypoints[0].velocity, 0, headings[0]));
            // Mark the first moment as constrained so that it cannot be changed
            constrained.insert(0);
        }
        else {
            moments.push_back(BasicMoment(0, 0, 0, headings[0]));
        }

        // Forwards pass
        for (int i = 1; i < params.sample_count; i++) {
            double dist = i * dpi;

            // Since the additional velocity constraints are sorted from shortest path length to
            // longest, we can check if we just surpassed one to determine whether we're on the
            // point. Then, remove it so the process still works.
            if (!constraints.empty() && dist >= constraints.front().first) {
                auto constraint = constraints.front();
                constraints.pop_front();
                // If the velocity is higher than the current, perform some extra checks and
                // computations
                if (constraint.second > moments[i - 1].vel) {
                    double accel = (constraint.second * constraint.second -
                                           moments[i - 1].vel * moments[i - 1].vel) /
                                   (2 * dpi);
                    if (accel > specs.max_a) {
                        throw std::invalid_argument("Waypoint velocity constraint cannot be met");
                    }
                    // Otherwise set accel and compute time diff
                    moments[i - 1].accel = accel;
                    time_diff[i - 1] = (constraint.second - moments[i - 1].vel) / accel;
                }
                // Ignore otherwise, it will be handled by the backwards pass

                // Make the new moment and mark it as constrained
                moments.push_back(BasicMoment(dist, constraint.second, 0, headings[i]));
                constrained.insert(i);
                continue;
            }

            // Otherwise do normal processing
            // Check if our velocity is less than the max at that point
            if (moments[i - 1].vel < mv[i]) {
                // Maybe improveable?
                // If we can accelerate then check the maximum velocity we can accelerate to
                double maxv =
                        std::sqrt(moments[i - 1].vel * moments[i - 1].vel + 2 * specs.max_a * dpi);
                double vel;
                if (maxv > mv[i]) {
                    // If it's more than the max then calculate the acceleration needed to reach the
                    // max
                    double accel =
                            (mv[i] * mv[i] - moments[i - 1].vel * moments[i - 1].vel) / (2 * dpi);
                    vel = mv[i];
                    moments[i - 1].accel = accel;
                }
                else {
                    // Otherwise set the velocity to be the max and set the previous moment's
                    // acceleration
                    vel = maxv;
                    moments[i - 1].accel = specs.max_a;
                }
                // Add the new moment and compute the time diff
                moments.push_back(BasicMoment(dist, vel, 0, headings[i]));
                // time diff computation is trivial since we can use the velocity differences
                time_diff[i - 1] = (vel - moments[i - 1].vel) / moments[i - 1].accel;
            }
            else {
                // If we can't accelerate just insert a normal moment with zero acceleration
                // The backwards pass will handle the rest
                moments.push_back(BasicMoment(dist, mv[i], 0, headings[i]));
            }
        }

        // Prepare for backwards pass by setting the last moment's data to the desired values
        moments[moments.size() - 1].accel = 0;
        moments[moments.size() - 1].vel = std::isnan(waypoints[waypoints.size() - 1].velocity)
                                                  ? 0
                                                  : waypoints[waypoints.size() - 1].velocity;
        // Backwards pass
        for (size_t i = moments.size() - 1; i-- > 0;) {
            // Only do processing if the velocity of this moment is greater than the next
            // i.e. deceleration is needed
            if (moments[i].vel > moments[i + 1].vel) {
                // Calculate max velocity like in the forwards pass but backwards this time
                double maxv =
                        std::sqrt(moments[i + 1].vel * moments[i + 1].vel + 2 * specs.max_a * dpi);

                double vel;
                // Compare with the velocity set by the forwards pass
                // If the velocity from the forwards pass is possible, then just set the
                // acceleration
                if (maxv > moments[i].vel) {
                    double accel = (moments[i].vel * moments[i].vel -
                                           moments[i + 1].vel * moments[i + 1].vel) /
                                   (2 * dpi);
                    moments[i].accel = -accel;
                    vel = moments[i].vel;
                }
                else {
                    // Otherwise, set deceleration to max
                    // If the moment is constrained, throw an exception
                    if (constrained.count(i)) {
                        throw std::invalid_argument("Waypoint velocity constraint cannot be met");
                    }
                    vel = maxv;
                    moments[i].accel = -specs.max_a;
                }

                moments[i].vel = vel;
                // Compute the time diff with the velocities
                time_diff[i] = (moments[i + 1].vel - vel) / moments[i].accel;
            }
        }

        // Set initial facing direction for all moments
        init_facing = moments[0].get_afacing();
        for (auto &moment : moments) {
            moment.init_facing = init_facing;
        }
        // Fill in the time for the moments
        for (size_t i = 1; i < moments.size(); i++) {
            // If we already have a time diff, then use that to calculate the next time
            if (!std::isnan(time_diff[i - 1])) {
                moments[i].time = moments[i - 1].time + time_diff[i - 1];
            }
            else {
                // If there is no time diff, it must mean that the acceleration is equal to zero
                // In this case we can simply use the position difference to calculate time
                // difference
                double dt = (moments[i].pos - moments[i - 1].pos) / moments[i - 1].vel;
                moments[i].time = moments[i - 1].time + dt;
            }
        }
    }

    std::pair<std::size_t, std::size_t> BasicTrajectory::search_moments(double t) const {
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

    BasicMoment BasicTrajectory::get(double t) const {
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

            BasicMoment moment(rpf::lerp(current.pos, next.pos, f),
                    rpf::lerp(current.vel, next.vel, f), rpf::lerp(current.accel, next.accel, f),
                    rpf::langle(current.heading, next.heading, f), t, init_facing);
            moment.backwards = backwards;
            return moment;
        }
    }

    Waypoint BasicTrajectory::get_pos(double t) const {
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

    std::shared_ptr<BasicTrajectory> BasicTrajectory::mirror_lr() const {
        auto p = path->mirror_lr();
        double ref = params.waypoints[0].heading;

        std::vector<BasicMoment> m;
        m.reserve(moments.size());

        for (size_t i = 0; i < moments.size(); i++) {
            BasicMoment moment(moments[i]);
            moment.heading = rpf::mangle(moment.heading, ref);
            moment.init_facing = params.waypoints[0].heading;
            m.push_back(moment);
        }
        return std::shared_ptr<BasicTrajectory>(
                new BasicTrajectory(p, std::move(m), backwards, specs, params));
    }
    std::shared_ptr<BasicTrajectory> BasicTrajectory::mirror_fb() const {
        auto p = path->mirror_fb();
        double ref = params.waypoints[0].heading + rpf::pi / 2;

        std::vector<BasicMoment> m;
        m.reserve(moments.size());
        for (size_t i = 0; i < moments.size(); i++) {
            BasicMoment moment(-moments[i].pos, -moments[i].vel, moments[i].accel,
                    rpf::mangle(moments[i].heading, ref), moments[i].time);
            moment.init_facing = params.waypoints[0].heading;
            moment.backwards = true;
            m.push_back(moment);
        }

        return std::shared_ptr<BasicTrajectory>(
                new BasicTrajectory(p, std::move(m), !backwards, specs, params));
    }
    std::shared_ptr<BasicTrajectory> BasicTrajectory::retrace() const {
        auto p = path->retrace();

        std::vector<BasicMoment> m;
        m.reserve(moments.size());
        auto &last = moments[moments.size() - 1];
        for (size_t i = 0; i < moments.size(); i++) {
            auto &current = moments[moments.size() - 1 - i];

            BasicMoment moment(-(last.pos - current.pos), -current.vel, current.accel,
                    -current.heading, last.time - current.time);
            moment.init_facing = params.waypoints[params.waypoints.size() - 1].heading;
            moment.backwards = true;
            m.push_back(moment);
        }

        return std::shared_ptr<BasicTrajectory>(
                new BasicTrajectory(p, std::move(m), !backwards, specs, params));
    }
} // namespace rpf
