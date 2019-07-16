#include "path/path.h"
#include "math/rpfmath.h"
#include "paths.h"
#include <cmath>
#include <stdexcept>
#include <string>

namespace rpf {

    Path::Path(const std::vector<Waypoint> &waypoints, double alpha, PathType type) : 
                waypoints(waypoints), alpha(alpha), type(type) {
        if(waypoints.size() < 2) {
            throw std::invalid_argument("Not enough waypoints");
        }
        segments.reserve(waypoints.size() - 1);
        switch(type) {
        case PathType::BEZIER:
            for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                segments.push_back(std::make_unique<BezierSegment>(BezierSegment::from_hermite(
                    static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                    Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                    Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)
                )));
            }
            break;
        case PathType::CUBIC_HERMITE:
            for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                segments.push_back(std::make_unique<CubicSegment>(
                    static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                    Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                    Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha)
                ));
            }
            break;
        case PathType::QUINTIC_HERMITE:
            for(size_t i = 0; i < waypoints.size() - 1; i ++) {
                segments.push_back(std::make_unique<QuinticSegment>(
                    static_cast<Vec2D>(waypoints[i]), static_cast<Vec2D>(waypoints[i + 1]),
                    Vec2D(std::cos(waypoints[i].heading) * alpha, std::sin(waypoints[i].heading) * alpha),
                    Vec2D(std::cos(waypoints[i + 1].heading) * alpha, std::sin(waypoints[i + 1].heading) * alpha),
                    Vec2D(0, 0), Vec2D(0, 0)
                ));
            }
            break;
        }
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
        double heading = std::atan2(deriv.y, deriv.x);
        double s = std::sin(heading);
        double c = std::cos(heading);

        std::pair<Vec2D, Vec2D> wheels;
        wheels.first = Vec2D(pos.x - (!backwards ? base_radius * s : -base_radius * s),
                pos.y + (!backwards ? base_radius * c : -base_radius * c));
        wheels.second = Vec2D(pos.x + (!backwards ? base_radius * s : -base_radius * s),
                pos.y - (!backwards ? base_radius * c : -base_radius * c));
        return wheels;
    }

    double Path::compute_len(int points) {
        double dt = 1.0 / (points - 1);

        Vec2D last = at(0);
        total_len = 0;
        s2t_table.push_back(std::pair<double, double>(0, 0));

        for(int i = 1; i < points; i ++) {
            Vec2D current = at(i * dt);
            total_len += last.dist(current);

            s2t_table.push_back(std::pair<double, double>(total_len, i * dt));
            last = current;
        }
        return total_len;
    }

    double Path::s2t(double s) const {
        if(s2t_table.size() == 0) {
            throw std::runtime_error("Lookup table not generated");
        }
        
        double dist = s * total_len;
        size_t start = 0;
        size_t end = s2t_table.size() - 1;
        size_t mid;

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

            double next = s2t_table[mid + 1].first;
            if(mid_dist <= dist && dist <= next) {
                double f = (dist - mid_dist) / (next - mid_dist);
                return rpf::lerp(s2t_table[mid].second, s2t_table[mid + 1].second, f);
            }
            if(mid == 0) {
                return 0;
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
        
        size_t start = 0;
        size_t end = s2t_table.size() - 1;
        size_t mid;

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
            
            double next = s2t_table[mid + 1].second;
            if(mid_t <= t && t <= next) {
                double f = (t - mid_t) / (next - mid_t);
                return rpf::lerp(s2t_table[mid].first, s2t_table[mid + 1].first, f) / total_len;
            }
            if(mid == 0) {
                return 0;
            }

            if(mid_t < t) {
                start = mid;
            }
            else {
                end = mid;
            }
        }
    }

    std::shared_ptr<Path> Path::mirror_lr() const {
        Vec2D ref(std::cos(waypoints[0].heading), std::sin(waypoints[0].heading));
        std::vector<Waypoint> w;
        w.reserve(waypoints.size());

        for(auto wp : waypoints) {
            w.push_back(Waypoint(static_cast<Vec2D>(wp).reflect(ref), rpf::mangle(wp.heading, waypoints[0].heading)));
        }
        auto p = std::make_shared<Path>(w, alpha, type);
        p->set_base(base_radius);
        return p;
    }
    std::shared_ptr<Path> Path::mirror_fb() const {
        Vec2D ref(-std::sin(waypoints[0].heading), std::cos(waypoints[0].heading));
        std::vector<Waypoint> w;
        w.reserve(waypoints.size());

        for(auto wp : waypoints) {
            w.push_back(Waypoint(static_cast<Vec2D>(wp).reflect(ref), rpf::mangle(wp.heading, waypoints[0].heading + rpf::pi / 2)));
        }
        auto p = std::make_shared<Path>(w, alpha, type);
        p->set_base(base_radius);
        p->set_backwards(!backwards);
        return p;
    }
    std::shared_ptr<Path> Path::retrace() const {
        std::vector<Waypoint> w;
        w.reserve(waypoints.size());

        for(auto rit = waypoints.rbegin(); rit != waypoints.rend(); ++rit) {
            Waypoint wp = *rit;
            w.push_back(Waypoint(wp.x, wp.y, rpf::rangle(wp.heading + rpf::pi)));
        }
        auto p = std::make_shared<Path>(w, alpha, type);
        p->set_base(base_radius);
        p->set_backwards(!backwards);
        return p;
    }

    std::shared_ptr<Path> Path::update(double t, const Vec2D &p, const Vec2D &v, const Vec2D &a) {
        if(waypoints.size() > 2) {
            throw std::invalid_argument("update() is not supported on paths with multiple segments!");
        }
        if(type != PathType::QUINTIC_HERMITE) {
            throw std::invalid_argument("update() is not supported for this path type!");
        }
        using namespace std::string_literals;
        if(t > 1.0 || t < 0.0) {
            throw std::invalid_argument("Time out of range: "s + std::to_string(t));
        }

        segments[0] = std::make_unique<QuinticSegment>(
            p, static_cast<Vec2D>(waypoints[1]),
            v, Vec2D(std::cos(waypoints[1].heading) * alpha, std::sin(waypoints[1].heading) * alpha),
            a, Vec2D(0, 0),
            t
        );
    }
}

