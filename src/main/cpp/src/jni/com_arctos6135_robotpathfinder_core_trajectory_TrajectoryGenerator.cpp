#include "jni/com_arctos6135_robotpathfinder_core_trajectory_TrajectoryGenerator.h"
#include "jni/jniutil.h"
#include "trajectory/tankdrivetrajectory.h"
#include "robotspecs.h"
#include "trajectoryparams.h"
#include "math/rpfmath.h"
#include <vector>
#include <list>
#include <memory>

extern std::list<std::shared_ptr<rpf::TankDriveTrajectory>> ttinstances;

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_TrajectoryGenerator__1generateRotationTank
        (JNIEnv *env, jclass clazz, jdouble maxv, jdouble maxa, jdouble base_width, jdouble angle) {
    double base_radius = base_width / 2;
    double distance = angle * base_radius;
    
    rpf::RobotSpecs specs(maxv, maxa, base_width);
    rpf::TrajectoryParams params;
    params.is_tank = true;
    params.type = rpf::PathType::BEZIER;
    params.sample_count = 100;
    params.waypoints.push_back(rpf::Waypoint(0, 0, rpf::pi / 2));
    params.waypoints.push_back(rpf::Waypoint(0, std::abs(distance), rpf::pi / 2));
    params.alpha = std::abs(distance) / 2;

    rpf::BasicTrajectory bt(specs, params);
    auto *t = new rpf::TankDriveTrajectory(bt);
    ttinstances.push_back(std::shared_ptr<rpf::TankDriveTrajectory>(t));

    auto &moments = t->get_moments();
    if(angle > 0) {
        for(auto &m : moments) {
            m.l_dist *= -1;
            m.l_vel *= -1;
            m.l_accel *= -1;
            m.heading = rpf::rangle(m.r_dist / base_radius + m.init_facing);
        }
    }
    else {
        for(auto &m : moments) {
            m.r_dist *= -1;
            m.r_vel *= -1;
            m.r_accel *= -1;
            m.heading = rpf::rangle(-m.l_dist / base_radius + m.init_facing);
        }
    }

    jclass tclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/JNITankDriveTrajectory");
    jmethodID mid = env->GetMethodID(tclass, "<init>", "(Lcom/arctos6135/robotpathfinder/core/RobotSpecs;Lcom/arctos6135/robotpathfinder/core/JNITrajectoryParams;J)V");
    return env->NewObject(tclass, mid, NULL, NULL, reinterpret_cast<jlong>(t));
}
