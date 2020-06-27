#include "math/rpfmath.h"

#include <cmath>

namespace rpf {
    double lerp(double a, double b, double f) {
        return (a * (1.0 - f)) + (b * f);
    }

    double restrict_angle(double angle) {
        angle = std::fmod(angle, pi * 2);
        if(angle <= -pi) {
            angle += pi * 2;
        }
        else if(angle > pi) {
            angle -= pi * 2;
        }
        return angle;
    }

    double mirror_angle(double angle, double ref) {
        return restrict_angle(angle - 2 * (angle - ref));
    }

    double lerp_angle(double a, double b, double f) {
        // Magic
        // https://stackoverflow.com/questions/2708476/rotation-interpolation
        double theta = std::fmod(std::fmod(a - b, pi * 2) + pi * 3, pi * 2) - pi;
        return restrict_angle(a + f * theta);
    }
    double lerp_angle(Vec2D a, Vec2D b, double f) {
        auto angle = Vec2D::lerp(a, b, f);
        return std::atan2(angle.y, angle.x);
    }

    double restrict_abs(double x, double m) {
        return std::abs(x) <= m ? x : std::copysign(m, x);
    }

    double curvature(double dx, double ddx, double dy, double ddy) {
        return (dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 3.0 / 2.0);
    }
} // namespace rpf
