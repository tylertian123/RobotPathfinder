#pragma once

#include "splinesegment.h"

namespace rpf {
    class QuinticSegment : public SplineSegment {
    public:
        QuinticSegment(const Vec2D &p0, const Vec2D &p1, const Vec2D &v0, const Vec2D &v1,
                const Vec2D &a0, const Vec2D &a1, double startT = 0);

        Vec2D at(double) const override;
        Vec2D deriv_at(double) const override;
        Vec2D second_deriv_at(double) const override;

    protected:
        double x_coeffs[6];
        double y_coeffs[6];
    };
} // namespace rpf
