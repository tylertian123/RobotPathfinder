#pragma once

#include "math/rpfmath.h"

namespace rpf {
    struct TankDriveMoment {

        TankDriveMoment() {}
        TankDriveMoment(double ld, double rd, double lv, double rv, double la, double ra, double h) 
                : l_dist(ld), r_dist(rd), l_vel(lv), r_vel(rv), l_accel(la), r_accel(ra), heading(h) {}
        TankDriveMoment(double ld, double rd, double lv, double rv, double la, double ra, double h, double t)
                : l_dist(ld), r_dist(rd), l_vel(lv), r_vel(rv), l_accel(la), r_accel(ra), heading(h), time(t) {}
        TankDriveMoment(double ld, double rd, double lv, double rv, double la, double ra, double h, double t, double ifacing)
                : l_dist(ld), r_dist(rd), l_vel(lv), r_vel(rv), l_accel(la), r_accel(ra), heading(h), time(t), init_facing(ifacing) {}
        
        double l_dist, r_dist;
        double l_vel, r_vel;
        double l_accel, r_accel;
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
}
