#include "segment/cubicsegment.h"
#include <cmath>

namespace rpf {
    // The basis functions
    double CubicSegment::basis0(double t) {
        return 2 * t * t * t - 3 * t * t + 1;
    }
    double CubicSegment::basis1(double t) {
        return t * t * t - 2 * t * t + t;
    }
    double CubicSegment::basis2(double t) {
        return -2 * t * t * t + 3 * t * t;
    }
    double CubicSegment::basis3(double t) {
        return t * t * t - t * t;
    }

    // The derivatives of the basis functions
    double CubicSegment::basis_deriv0(double t) {
        return 6 * t * t - 6 * t;
    }
    double CubicSegment::basis_deriv1(double t) {
        return 3 * t * t - 4 * t + 1;
    }
    double CubicSegment::basis_deriv2(double t) {
        return -6 * t * t + 6 * t;
    }
    double CubicSegment::basis_deriv3(double t) {
        return 3 * t * t - 2 * t;
    }

    // The second derivatives of the basis functions
    double CubicSegment::basis_second_deriv0(double t) {
        return 12 * t - 6;
    }
    double CubicSegment::basis_second_deriv1(double t) {
        return 6 * t - 4;
    }
    double CubicSegment::basis_second_deriv2(double t) {
        return -12 * t + 6;
    }
    double CubicSegment::basis_second_deriv3(double t) {
        return 6 * t - 2;
    }

    Vec2D CubicSegment::at(double t) const {
        return p0 * basis0(t) + m0 * basis1(t) + p1 * basis2(t) + m1 * basis3(t);
    }
    Vec2D CubicSegment::deriv_at(double t) const {
        return p0 * basis_deriv0(t) + m0 * basis_deriv1(t) + p1 * basis_deriv2(t) +
               m1 * basis_deriv3(t);
    }
    Vec2D CubicSegment::second_deriv_at(double t) const {
        return p0 * basis_second_deriv0(t) + m0 * basis_second_deriv1(t) +
               p1 * basis_second_deriv2(t) + m1 * basis_second_deriv3(t);
    }
} // namespace rpf
