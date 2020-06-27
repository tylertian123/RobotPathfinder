#pragma once

#include "splinesegment.h"

namespace rpf {
    class BezierSegment : public SplineSegment {
    public:
        BezierSegment(const Vec2D &a, const Vec2D &b, const Vec2D &c, const Vec2D &d) {
            ctrl_pts[0] = a;
            ctrl_pts[1] = b;
            ctrl_pts[2] = c;
            ctrl_pts[3] = d;
        }

        virtual Vec2D at(double) const override;
        virtual Vec2D deriv_at(double) const override;
        virtual Vec2D second_deriv_at(double) const override;

        static BezierSegment from_hermite(
                const Vec2D &, const Vec2D &, const Vec2D &, const Vec2D &);

    protected:
        Vec2D ctrl_pts[4];
    };
} // namespace rpf
