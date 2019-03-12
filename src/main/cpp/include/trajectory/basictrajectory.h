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
        BasicTrajectory(const RobotSpecs &specs, const TrajectoryParams &params) : specs(specs), params(params) {
            path = std::make_shared<Path>(params.waypoints, params.alpha, params.type);
            auto &waypoints = params.waypoints;

            if(params.is_tank) {
                path->set_base(specs.base_width / 2);
            }
            double ds = 1.0 / (params.sample_count - 1);
            double total = path->compute_len(params.sample_count);
            double dpi = total / (params.sample_count - 1);
            
            std::vector<double> mv;
            mv.reserve(params.sample_count);
            std::list<std::pair<double, double>> constraints;

            double wpdt = 1.0 / (waypoints.size() - 1);
            for(size_t i = 1; i < waypoints.size() - 1; i ++) {
                if(!std::isnan(waypoints[i].velocity)) {
                    constraints.push_back(std::make_pair(path->t2s(i * wpdt) * total, waypoints[i].velocity));
                }
            }
            
            std::vector<double> headings;
            headings.reserve(params.sample_count);
            patht.reserve(params.sample_count);
            pathr.reserve(params.sample_count);
            moments.reserve(params.sample_count);

            if(params.is_tank) {
                for(int i = 0; i < params.sample_count; i ++) {
                    double t = path->s2t(ds * i);
                    patht.push_back(t);
                    
                    auto d = path->deriv_at(t);
                    auto dd = path->second_deriv_at(t);
                    double curvature = rpf::curvature(d.x, dd.x, d.y, dd.y);

                    headings.push_back(std::atan2(d.y, d.x));
                    
                    pathr.push_back(1 / curvature);
                    mv.push_back(specs.max_v / (1 + specs.base_width / (2 * std::abs(pathr[i]))));
                }
            }
            else {
                for(int i = 0; i < params.sample_count; i ++) {
                    mv.push_back(specs.max_v);

                    double t = path->s2t(ds * i);
                    Vec2D d = path->deriv_at(t);
                    headings.push_back(std::atan2(d.y, d.x));
                }
            }

            if(!std::isnan(waypoints[0].velocity)) {
                moments.push_back(BasicMoment(0, waypoints[0].velocity, 0, headings[0]));
            }
            else {
                moments.push_back(BasicMoment(0, 0, 0, headings[0]));
            }

            std::vector<double> time_diff(params.sample_count - 1, std::numeric_limits<double>::quiet_NaN());
            std::unordered_set<int> constrained;

            for(int i = 1; i < params.sample_count; i ++) {
                double dist = i * dpi;

                if(!constraints.empty() && dist >= constraints.front().first) {
                    auto constraint = constraints.front();
                    constraints.pop_front();

                    if(constraint.second > moments[i - 1].vel) {
                        double accel = constraint.second * constraint.second - moments[i - 1].vel * moments[i - 1].vel;
                        if(accel > specs.max_a) {
                            throw std::invalid_argument("Waypoint velocity constraint cannot be met");
                        }
                        moments[i - 1].accel = accel;
                        time_diff[i - 1] = (constraint.second - moments[i - 1].vel) / accel;
                    }
                    else {
                        moments[i - 1].accel = 0;
                    }
                    moments.push_back(BasicMoment(dist, constraint.second, 0, headings[i]));
                    constrained.insert(i);
                    continue;
                }

                if(moments[i - 1].vel < mv[i]) {
                    // Maybe improveable?
                    double maxv = std::sqrt(moments[i - 1].vel * moments[i - 1].vel + 2 * specs.max_a * dpi);
                    double vel;

                    if(maxv > mv[i]) {
                        double accel = (mv[i] * mv[i] - moments[i - 1].vel * moments[i - 1].vel) / (2 * dpi);
                        vel = mv[i];
                        moments[i - 1].accel = accel;
                    }
                    else {
                        vel = maxv;
                        moments[i - 1].accel = specs.max_a;
                    }

                    moments.push_back(BasicMoment(dist, vel, 0, headings[i]));
                    time_diff[i - 1] = (vel - moments[i - 1].vel) / moments[i - 1].accel;
                }
                else {
                    moments.push_back(BasicMoment(dist, mv[i], 0, headings[i]));
                    moments[i - 1].accel = 0;
                }
            }

            moments[moments.size() - 1].accel = 0;
            moments[moments.size() - 1].vel = std::isnan(waypoints[waypoints.size() - 1].velocity) ? 0 : waypoints[waypoints.size() - 1].velocity;

            for(size_t i = moments.size() - 1; i --> 0; ) {
                if(moments[i].vel > moments[i + 1].vel) {
                    double maxv = std::sqrt(moments[i + 1].vel * moments[i + 1].vel + 2 * specs.max_a * dpi);

                    double vel;
                    if(maxv > moments[i].vel) {
                        double accel = (moments[i].vel * moments[i].vel - moments[i + 1].vel * moments[i + 1].vel) / (2 * dpi);
                        moments[i].accel = -accel;
                        vel = moments[i].vel;
                    }
                    else {
                        if(constrained.count(i)) {
                            throw std::invalid_argument("Waypoint velocity constraint cannot be met");
                        }
                        vel = maxv;
                        moments[i].accel = -specs.max_a;
                    }

                    moments[i].vel = vel;
                    time_diff[i] = (moments[i + 1].vel - vel) / moments[i].accel;

                }
            }

            init_facing = moments[0].get_afacing();
            for(auto &moment : moments) {
                moment.init_facing = init_facing;
            }

            for(size_t i = 1; i < moments.size(); i ++) {
                if(!std::isnan(time_diff[i - 1])) {
                    moments[i].time = moments[i - 1].time + time_diff[i - 1];
                }
                else {
                    double dt = (moments[i].dist - moments[i - 1].dist) / moments[i - 1].vel;
                    moments[i].time = moments[i - 1].time + dt;
                }
            }
        }

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
