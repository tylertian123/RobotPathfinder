#include "path.h"
#include <cmath>

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
}

