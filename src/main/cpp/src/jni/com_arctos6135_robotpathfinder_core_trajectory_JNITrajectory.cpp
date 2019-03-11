#include "jni/com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory.h"
#include "jni/jniutil.h"
#include "trajectory/basictrajectory.h"
#include <vector>
#include <memory>
#include <algorithm>

std::vector<std::shared_ptr<rpf::BasicTrajectory>> btinstances;

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory__1construct
        (JNIEnv *env, jobject obj, jdouble maxv, jdouble maxa, jdouble base_width, jboolean is_tank, jobjectArray waypoints, jdouble alpha, jint segment_count, jint type) {
    std::vector<rpf::Waypoint> wp;
    wp.reserve(env->GetArrayLength(waypoints));
    // Translate the waypoints into C++ ones
    for(int i = 0; i < env->GetArrayLength(waypoints); i ++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        wp.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"), rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading"), rpf::get_field<double>(env, waypoint, "velocity")));
    }

    rpf::RobotSpecs specs(maxv, maxa, base_width);
    rpf::TrajectoryParams params;
    params.waypoints = std::move(wp);
    params.is_tank = is_tank;
    params.seg_count = segment_count;
    params.type = static_cast<rpf::PathType>(type);

    rpf::BasicTrajectory *t = new rpf::BasicTrajectory(specs, params);
    btinstances.push_back(std::shared_ptr<rpf::BasicTrajectory>(t));
    rpf::set_obj_ptr(env, obj, t);
}
