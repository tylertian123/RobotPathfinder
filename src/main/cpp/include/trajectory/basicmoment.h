#pragma once

#include <limits>
#include "math/rpfmath.h"

namespace rpf {
    struct BasicMoment {

        BasicMoment() {}
        BasicMoment(double d, double v, double a, double h, double initf) : dist(d), vel(v), accel(a), init_facing(initf) {}
        BasicMoment(double d, double v, double a, double h) : dist(d), vel(v), accel(a), init_facing(std::numeric_limits<double>::quiet_NaN()) {}
        
        double dist;
        double vel;
        double accel;
        double heading;
        double time;

        double init_facing;

        double get_afacing() {
            if(vel > 0) {
                return rangle(heading);
            }
            else if(vel < 0) {
                return rangle(heading + rpf::pi);
            }
            else {
                return rangle(accel >= 0 ? heading : heading + rpf::pi);
            }
        }
        double get_rfacing() {
            return rangle(get_afacing() - init_facing);
        }
    };
}
