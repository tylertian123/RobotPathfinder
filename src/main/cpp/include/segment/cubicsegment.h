#pragma once

#include "splinesegment.h"

namespace rpf {
    class CubicSegment : public SplineSegment {
    public:
        CubicSegment(const Vec2D &p0, const Vec2D &p1, const Vec2D &m0, const Vec2D &m1) : p0(p0), p1(p1), m0(m0), m1(m1) {}

        Vec2D at(double) const override;
        Vec2D deriv_at(double) const override;
        Vec2D second_deriv_at(double) const override;

    protected:
        static double basis0(double);
        static double basis1(double);
        static double basis2(double);
        static double basis3(double);

        static double basis_deriv0(double);
        static double basis_deriv1(double);
        static double basis_deriv2(double);
        static double basis_deriv3(double);

        static double basis_second_deriv0(double);
        static double basis_second_deriv1(double);
        static double basis_second_deriv2(double);
        static double basis_second_deriv3(double);

        Vec2D p0, p1, m0, m1;
    };
}
