#include "segment/beziersegment.h"

namespace rpf {
    BezierSegment BezierSegment::from_hermite(const Vec2D &at0, const Vec2D &at1, 
            const Vec2D &deriv_at0, const Vec2D &deriv_at1) {
        Vec2D p1 = at0 + deriv_at0 * (1.0 / 3.0);
        Vec2D p2 = at1 + deriv_at1 * (-1.0 / 3.0);
        return BezierSegment(at0, p1, p2, at1);
    }

    Vec2D BezierSegment::at(double t) const {
        double u = 1 - t;
		double uu = u * u;
		double uuu = u * u * u;
		double tt = t * t;
		double ttt = t * t * t;
        return ctrl_pts[0] * uuu + ctrl_pts[1] * 3 * uu * t + ctrl_pts[2] * 3 * u * tt + ctrl_pts[3] * ttt;
    }

    Vec2D BezierSegment::deriv_at(double t) const {
        double u = 1 - t;
		double uu = u * u;
		double tt = t * t;
        return (ctrl_pts[1] - ctrl_pts[0]) * 3 * uu + (ctrl_pts[2] - ctrl_pts[1]) * 6 * u * t +
                (ctrl_pts[3] - ctrl_pts[2]) * 3 * tt;
    }

    Vec2D BezierSegment::second_deriv_at(double t) const {
        double u = 1 - t;
        return (ctrl_pts[2] - 2 * ctrl_pts[1] + ctrl_pts[0]) * 6 * u + (ctrl_pts[3] - 2 * ctrl_pts[2] + ctrl_pts[1]) * 6 * t;
    }
}
