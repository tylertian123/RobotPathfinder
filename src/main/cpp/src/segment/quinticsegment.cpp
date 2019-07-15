#include "segment/quinticsegment.h"
#include <cmath>
#include "math/mat.h"

namespace rpf {
    
	QuinticSegment::QuinticSegment(const Vec2D &p0, const Vec2D &p1, const Vec2D &v0, const Vec2D &v1, const Vec2D &a0, 
			const Vec2D &a1, double startT) {
		
		Mat<double> mat(6, 7);

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
		
		mat[0][0] = 1;
		mat[0][1] = x;
		mat[0][2] = x2;
		mat[0][3] = x3;
		mat[0][4] = x4;
		mat[0][5] = x5;

		mat[1][0] = 0;
		mat[1][1] = 1;
		mat[1][2] = 2 * x;
		mat[1][3] = 3 * x2;
		mat[1][4] = 4 * x3;
		mat[1][5] = 5 * x4;

		mat[2][0] = 0;
		mat[2][1] = 0;
		mat[2][2] = 2;
		mat[2][3] = 6 * x;
		mat[2][4] = 12 * x2;
		mat[2][5] = 20 * x3;

		mat[3][0] = 1;
		mat[3][1] = 1;
		mat[3][2] = 1;
		mat[3][3] = 1;
		mat[3][4] = 1;
		mat[3][5] = 1;

		mat[4][0] = 0;
		mat[4][1] = 1;
		mat[4][2] = 2;
		mat[4][3] = 3;
		mat[4][4] = 4;
		mat[4][5] = 5;
		
		mat[5][0] = 0;
		mat[5][1] = 0;
		mat[5][2] = 2;
		mat[5][3] = 6;
		mat[5][4] = 12;
		mat[5][5] = 20;

		mat[0][6] = p0.x;
		mat[1][6] = v0.x;
		mat[2][6] = a0.x;
		mat[3][6] = p1.x;
		mat[4][6] = v1.x;
		mat[5][6] = a1.x;

		mat.eliminate();
		for(int i = 0; i < 6; i ++) {
			x_coeffs[i] = mat[i][6];
		}

		mat[0][0] = 1;
		mat[0][1] = x;
		mat[0][2] = x2;
		mat[0][3] = x3;
		mat[0][4] = x4;
		mat[0][5] = x5;

		mat[1][0] = 0;
		mat[1][1] = 1;
		mat[1][2] = 2 * x;
		mat[1][3] = 3 * x2;
		mat[1][4] = 4 * x3;
		mat[1][5] = 5 * x4;

		mat[2][0] = 0;
		mat[2][1] = 0;
		mat[2][2] = 2;
		mat[2][3] = 6 * x;
		mat[2][4] = 12 * x2;
		mat[2][5] = 20 * x3;

		mat[3][0] = 1;
		mat[3][1] = 1;
		mat[3][2] = 1;
		mat[3][3] = 1;
		mat[3][4] = 1;
		mat[3][5] = 1;

		mat[4][0] = 0;
		mat[4][1] = 1;
		mat[4][2] = 2;
		mat[4][3] = 3;
		mat[4][4] = 4;
		mat[4][5] = 5;
		
		mat[5][0] = 0;
		mat[5][1] = 0;
		mat[5][2] = 2;
		mat[5][3] = 6;
		mat[5][4] = 12;
		mat[5][5] = 20;

		mat[0][6] = p0.y;
		mat[1][6] = v0.y;
		mat[2][6] = a0.y;
		mat[3][6] = p1.y;
		mat[4][6] = v1.y;
		mat[5][6] = a1.y;

		mat.eliminate();
		for(int i = 0; i < 6; i ++) {
			y_coeffs[i] = mat[i][6];
		}
	}

	Vec2D QuinticSegment::at(double t) const {
		double t2 = t * t;
		double t3 = t2 * t;
		double t4 = t3 * t;
		double t5 = t4 * t;
		
		return Vec2D(
			x_coeffs[0] + x_coeffs[1] * t + x_coeffs[2] * t2 + x_coeffs[3] * t3 + x_coeffs[4] * t4 + x_coeffs[5] * t5,
			y_coeffs[0] + y_coeffs[1] * t + y_coeffs[2] * t2 + y_coeffs[3] * t3 + y_coeffs[4] * t4 + y_coeffs[5] * t5
		);
	}

	Vec2D QuinticSegment::deriv_at(double t) const {
		double t2 = t * t;
		double t3 = t2 * t;
		double t4 = t3 * t;

		return Vec2D(
			x_coeffs[1] + 2 * x_coeffs[2] * t + 3 * x_coeffs[3] * t2 + 4 * x_coeffs[4] * t3 + 5 * x_coeffs[5] * t4,
			y_coeffs[1] + 2 * y_coeffs[2] * t + 3 * y_coeffs[3] * t2 + 4 * y_coeffs[4] * t3 + 5 * y_coeffs[5] * t4
		);
	}

	Vec2D QuinticSegment::second_deriv_at(double t) const {
		double t2 = t * t;
		double t3 = t2 * t;

		return Vec2D(
			2 * x_coeffs[2] + 6 * x_coeffs[3] * t + 12 * x_coeffs[4] * t2 + 20 * x_coeffs[5] * t3,
			2 * y_coeffs[2] + 6 * y_coeffs[3] * t + 12 * y_coeffs[4] * t2 + 20 * y_coeffs[5] * t3
		);
	}
}
