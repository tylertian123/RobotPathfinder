#include "segment/quinticsegment.h"
#include "math/mat.h"
#include <cmath>

namespace rpf {

    QuinticSegment::QuinticSegment(const Vec2D &p0, const Vec2D &p1, const Vec2D &v0,
            const Vec2D &v1, const Vec2D &a0, const Vec2D &a1, double startT) {
        /*
         * The matrix is used to solve this nasty system of equations:
         *
         * b0 + xb1 + x^2b2 +  x^3b3 +   x^4b4 +   x^5b5 = p0
         *       b1 +  2xb2 + 3x^2b3 +  4x^3b4 +  5x^4b5 = v0
         *              2b2 +   6xb3 + 12x^2b4 + 20x^3b5 = a0
         * b0 +  b1 +    b2 +     b3 +      b4 +      b5 = p1
         *       b1 +   2b2 +    3b3 +     4b4 +     5b5 = v1
         *              2b2 +    6b3 +    12b4 +    20b5 = a1
         */

        double x = startT;
        double x2 = x * startT;
        double x3 = x2 * startT;
        double x4 = x3 * startT;
        double x5 = x4 * startT;

        Mat<double> mat = {
                {1, x, x2, x3, x4, x5, p0.x},
                {0, 1, 2 * x, 3 * x2, 4 * x3, 5 * x4, v0.x},
                {0, 0, 2, 6 * x, 12 * x2, 20 * x3, a0.x},
                {1, 1, 1, 1, 1, 1, p1.x},
                {0, 1, 2, 3, 4, 5, v1.x},
                {0, 0, 2, 6, 12, 20, a1.x},
        };
        mat.eliminate();
        for (int i = 0; i < 6; i++) {
            x_coeffs[i] = mat[i][6];
        }

        mat = {
                {1, x, x2, x3, x4, x5, p0.y},
                {0, 1, 2 * x, 3 * x2, 4 * x3, 5 * x4, v0.y},
                {0, 0, 2, 6 * x, 12 * x2, 20 * x3, a0.y},
                {1, 1, 1, 1, 1, 1, p1.y},
                {0, 1, 2, 3, 4, 5, v1.y},
                {0, 0, 2, 6, 12, 20, a1.y},
        };
        mat.eliminate();
        for (int i = 0; i < 6; i++) {
            y_coeffs[i] = mat[i][6];
        }
    }

    Vec2D QuinticSegment::at(double t) const {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        return Vec2D(x_coeffs[0] + x_coeffs[1] * t + x_coeffs[2] * t2 + x_coeffs[3] * t3 +
                             x_coeffs[4] * t4 + x_coeffs[5] * t5,
                y_coeffs[0] + y_coeffs[1] * t + y_coeffs[2] * t2 + y_coeffs[3] * t3 +
                        y_coeffs[4] * t4 + y_coeffs[5] * t5);
    }

    Vec2D QuinticSegment::deriv_at(double t) const {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;

        return Vec2D(x_coeffs[1] + 2 * x_coeffs[2] * t + 3 * x_coeffs[3] * t2 +
                             4 * x_coeffs[4] * t3 + 5 * x_coeffs[5] * t4,
                y_coeffs[1] + 2 * y_coeffs[2] * t + 3 * y_coeffs[3] * t2 + 4 * y_coeffs[4] * t3 +
                        5 * y_coeffs[5] * t4);
    }

    Vec2D QuinticSegment::second_deriv_at(double t) const {
        double t2 = t * t;
        double t3 = t2 * t;

        return Vec2D(2 * x_coeffs[2] + 6 * x_coeffs[3] * t + 12 * x_coeffs[4] * t2 +
                             20 * x_coeffs[5] * t3,
                2 * y_coeffs[2] + 6 * y_coeffs[3] * t + 12 * y_coeffs[4] * t2 +
                        20 * y_coeffs[5] * t3);
    }
} // namespace rpf
