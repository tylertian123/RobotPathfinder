#include "quinticsegment.h"
#include <cmath>

namespace rpf {
    // The 6 quintic hermite basis functions
	// They can be found here: https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
	double QuinticSegment::basis0(double t) {
		return 1 - 10 * t * t * t + 15 * std::pow(t, 4) - 6 * std::pow(t, 5);
	}
	double QuinticSegment::basis1(double t) {
		return t - 6 * t * t * t + 8 * std::pow(t, 4) - 3 * std::pow(t, 5);
	}
	double QuinticSegment::basis2(double t) {
		return t * t / 2 - 3 * t * t * t / 2 + 3 * std::pow(t, 4) / 2 - std::pow(t, 5) / 2;
	}
	double QuinticSegment::basis3(double t) {
		return t * t * t / 2 - std::pow(t, 4) + std::pow(t, 5) / 2;
	}
	double QuinticSegment::basis4(double t) {
		return -4 * t * t * t + 7 * std::pow(t, 4) - 3 * std::pow(t, 5);
	}
	double QuinticSegment::basis5(double t) {
		return 10 * t * t * t - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
	}
	
	// The derivatives of the basis functions
	double QuinticSegment::basis_deriv0(double t) {
		return -30 * t * t + 60 * t * t * t - 30 * std::pow(t, 4);
	}
	double QuinticSegment::basis_deriv1(double t) {
		return 1 - 18 * t * t + 32 * t * t * t - 15 * std::pow(t, 4);
	}
	double QuinticSegment::basis_deriv2(double t) {
		return t - 9 * t * t / 2 + 6 * t * t * t - 5 * std::pow(t, 4) / 2;
	}
	double QuinticSegment::basis_deriv3(double t) {
		return 3 * t * t / 2 - 4 * t * t * t + 5 * std::pow(t,  4) / 2;
	}
	double QuinticSegment::basis_deriv4(double t) {
		return -12 * t * t + 28 * t * t * t - 15 * std::pow(t, 4);
	}
	double QuinticSegment::basis_deriv5(double t) {
		return 30 * t * t - 60 * t * t * t + 30 * std::pow(t, 4);
	}
	
	// The second derivatives of the basis functions
	double QuinticSegment::basis_second_deriv0(double t) {
		return -60 * t + 180 * t * t - 120 * t * t * t;
	}
	double QuinticSegment::basis_second_deriv1(double t) {
		return -36 * t + 96 * t * t - 60 * t * t * t;
	}
	double QuinticSegment::basis_second_deriv2(double t) {
		return 1 - 9 * t + 18 * t * t - 10 * t * t * t;
	}
	double QuinticSegment::basis_second_deriv3(double t) {
		return 3 * t - 12 * t * t + 10 * t * t * t;
	}
	double QuinticSegment::basis_second_deriv4(double t) {
		return -24 * t + 84 * t * t - 60 * t * t * t;
	}
	double QuinticSegment::basis_second_deriv5(double t) {
		return 60 * t - 180 * t * t + 120 * t * t * t;
	}

    Vec2D QuinticSegment::at(double t) const {
        return p0 * basis0(t) + v0 * basis1(t) + a0 * basis2(t) + a1 * basis3(t) + v1 * basis4(t) + p1 * basis5(t);
    }
    Vec2D QuinticSegment::deriv_at(double t) const {
        return p0 * basis_deriv0(t) + v0 * basis_deriv1(t) + a0 * basis_deriv2(t) + a1 * basis_deriv3(t) + v1 * basis_deriv4(t) + p1 * basis_deriv5(t);
    }
    Vec2D QuinticSegment::second_deriv_at(double t) const {
        return p0 * basis_second_deriv0(t) + v0 * basis_second_deriv1(t) + a0 * basis_second_deriv2(t) + a1 * basis_second_deriv3(t) + v1 * basis_second_deriv4(t) + p1 * basis_second_deriv5(t);
    }
}
