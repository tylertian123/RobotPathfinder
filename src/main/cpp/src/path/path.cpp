#include "path/path.h"
#include "math/rpfmath.h"
#include "paths.h"
#include <cmath>
#include <stdexcept>

namespace rpf {
    void Path::set_base(double base_radius) {
        this->base_radius = base_radius;
    }
    double Path::get_base() const {
        return base_radius;
    }

    Vec2D Path::at(double t) const {
        if(t >= 1) {
            return segments[segments.size() - 1]->at(1);
        }

        t *= segments.size();
        return segments[(size_t) std::floor(t)]->at(std::fmod(t, 1.0));
    }
    Vec2D Path::deriv_at(double t) const {
        if(t >= 1) {
            return segments[segments.size() - 1]->deriv_at(1);
        }

        t *= segments.size();
        return segments[(size_t) std::floor(t)]->deriv_at(std::fmod(t, 1.0));
    }
    Vec2D Path::second_deriv_at(double t) const {
        if(t >= 1) {
            return segments[segments.size() - 1]->second_deriv_at(1);
        }

        t *= segments.size();
        return segments[(size_t) std::floor(t)]->second_deriv_at(std::fmod(t, 1.0));
    }
    std::pair<Vec2D, Vec2D> Path::wheels_at(double t) const {

        Vec2D pos = at(t);
        Vec2D deriv = deriv_at(t);
        double heading = std::atan2(deriv.get_y(), deriv.get_x());
        double s = std::sin(heading);
        double c = std::cos(heading);

        std::pair<Vec2D, Vec2D> wheels;
        wheels.first = Vec2D(pos.get_x() - (!backwards ? base_radius * s : -base_radius * s),
                pos.get_y() + (!backwards ? base_radius * c : -base_radius * c));
        wheels.second = Vec2D(pos.get_y() + (!backwards ? base_radius * s : -base_radius * s),
                pos.get_y() - (!backwards ? base_radius * c : -base_radius * c));
        return wheels;
    }

    double Path::compute_len(int points) {
        double dt = 1.0 / (points - 1);

        Vec2D last = at(0);
        total_len = 0;
        s2t_table.resize(points);
        s2t_table.push_back(std::pair<double, double>(0, 0));

        for(int i = 1; i < points; i ++) {
            Vec2D current = at(i * dt);
            total_len += last.dist(current);

            s2t_table.push_back(std::pair<double, double>(total_len, i * dt));
            last = current;
        }
        return total_len;
    }
    double Path::get_len() const {
        return total_len;
    }

    double Path::s2t(double s) const {
        if(s2t_table.size() == 0) {
            throw std::runtime_error("Lookup table not generated");
        }
        
        double dist = s * total_len;
        int start = 0;
        int end = s2t_table.size() - 1;
        int mid;

        if(dist > s2t_table[end - 1].first) {
            return 1;
        }
        while(true) {
            mid = (start + end) / 2;
            double mid_dist = s2t_table[mid].first;

            if(mid_dist == dist) {
                return s2t_table[mid].second;
            }
            if(mid == s2t_table.size() - 1) {
                return 1;
            }
            if(mid == 0) {
                return 0;
            }

            double next = s2t_table[mid + 1].first;
            if(mid_dist <= dist && dist <= next) {
                double f = (dist - mid_dist) / (next - mid_dist);
                return rpf::lerp(s2t_table[mid].second, s2t_table[mid + 1].second, f);
            }
            
            if(mid_dist < dist) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }
    double Path::t2s(double t) const {
        if(s2t_table.size() == 0) {
            throw std::runtime_error("Lookup table not generated");
        }
        
        int start = 0;
        int end = s2t_table.size() - 1;
        int mid;

        if(t >= 1) {
            return 1;
        }
        while(true) {
            mid = (start + end) / 2;
            double mid_t = s2t_table[mid].second;

            if(mid_t == t) {
                return s2t_table[mid].first / total_len;
            }
            if(mid == s2t_table.size() - 1) {
                return 1;
            }
            if(mid == 0) {
                return 0;
            }
            
            double next = s2t_table[mid + 1].second;
            if(mid_t <= t && t <= next) {
                double f = (t - mid_t) / (next - mid_t);
                return rpf::lerp(s2t_table[mid].first, s2t_table[mid + 1].first, f) / total_len;
            }

            if(mid_t < t) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }

    double Path::get_alpha() const {
        return alpha;
    }
    PathType Path::get_type() const {
        return type;
    }
    std::vector<Waypoint> Path::get_waypoints() const {
        return waypoints;
    }
    bool Path::get_backwards() const {
        return backwards;
    }
    void Path::set_backwards(bool backwards) {
        this->backwards = backwards;
    }

    std::shared_ptr<Path> Path::mirror_lr() const {
        Vec2D ref(std::cos(waypoints[0].heading), std::sin(waypoints[0].heading));
        std::vector<Waypoint> w(waypoints.size());

        for(auto wp : waypoints) {
            w.push_back(Waypoint(static_cast<Vec2D>(wp).reflect(ref), rpf::mangle(wp.heading, waypoints[0].heading)));
        }
        auto p = construct_path(w, alpha, type);
        p->set_base(base_radius);
        return p;
    }
    std::shared_ptr<Path> Path::mirror_fb() const {
        Vec2D ref(-std::sin(waypoints[0].heading), std::cos(waypoints[0].heading));
        std::vector<Waypoint> w(waypoints.size());

        for(auto wp : waypoints) {
            w.push_back(Waypoint(static_cast<Vec2D>(wp).reflect(ref), rpf::mangle(wp.heading, waypoints[0].heading + rpf::pi / 2)));
        }
        auto p = construct_path(w, alpha, type);
        p->set_base(base_radius);
        p->set_backwards(!backwards);
        return p;
    }
    std::shared_ptr<Path> Path::retrace() const {
        std::vector<Waypoint> w(waypoints.size());

        for(auto rit = waypoints.rbegin(); rit != waypoints.rend(); ++rit) {
            Waypoint wp = *rit;
            w.push_back(Waypoint(wp.x, wp.y, rpf::rangle(wp.heading + rpf::pi)));
        }
        auto p = construct_path(w, alpha, type);
        p->set_base(base_radius);
        p->set_backwards(!backwards);
        return p;
    }

    static std::shared_ptr<Path> construct_path(const std::vector<Waypoint> &waypoints, double alpha, PathType type) {
        switch(type) {
        case PathType::BEZIER:
            return std::make_shared<BezierPath>(waypoints, alpha);
        case PathType::CUBIC_HERMITE:
            return std::make_shared<CubicPath>(waypoints, alpha);
        case PathType::QUINTIC_HERMITE:
            return std::make_shared<QuinticPath>(waypoints, alpha);
        }
    }
}

