#pragma once

#include "math/rpfmath.h"
#include <limits>

namespace rpf {
    struct BasicMoment {

        BasicMoment() {
        }
        BasicMoment(double d, double v, double a, double h, double t)
                : pos(d), vel(v), accel(a), heading(h), time(t) {
        }
        BasicMoment(double d, double v, double a, double h, double t, double initf)
                : pos(d), vel(v), accel(a), heading(h), time(t), init_facing(initf) {
        }
        BasicMoment(double d, double v, double a, double h)
                : pos(d), vel(v), accel(a), heading(h),
                  init_facing(std::numeric_limits<double>::quiet_NaN()) {
        }

        double pos;
        double vel;
        double accel;
        double heading;
        double time;

        double init_facing;
        bool backwards = false;

        double get_afacing() {
            return backwards ? -heading : heading;
        }
        double get_rfacing() {
            return rangle(get_afacing() - init_facing);
        }
    };
} // namespace rpf
