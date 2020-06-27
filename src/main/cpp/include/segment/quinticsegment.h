#pragma once

#include "splinesegment.h"

namespace rpf {
    class QuinticSegment : public SplineSegment {
    public:
        QuinticSegment(const Vec2D &p0, const Vec2D &p1, const Vec2D &v0, const Vec2D &v1,
                const Vec2D &a0, const Vec2D &a1)
                : p0(p0), p1(p1), v0(v0), v1(v1), a0(a0), a1(a1) {
        }

        Vec2D at(double) const override;
        Vec2D deriv_at(double) const override;
        Vec2D second_deriv_at(double) const override;

    protected:
        static double basis0(double);
        static double basis1(double);
        static double basis2(double);
        static double basis3(double);
        static double basis4(double);
        static double basis5(double);

        static double basis_deriv0(double);
        static double basis_deriv1(double);
        static double basis_deriv2(double);
        static double basis_deriv3(double);
        static double basis_deriv4(double);
        static double basis_deriv5(double);

        static double basis_second_deriv0(double);
        static double basis_second_deriv1(double);
        static double basis_second_deriv2(double);
        static double basis_second_deriv3(double);
        static double basis_second_deriv4(double);
        static double basis_second_deriv5(double);

        Vec2D p0, p1, v0, v1, a0, a1;
    };
} // namespace rpf
