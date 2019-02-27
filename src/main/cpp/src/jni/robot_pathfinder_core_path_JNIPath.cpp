#include "robot_pathfinder_core_path_JNIPath.h"
#include "paths.h"
#include "jni/jniutil.h"
#include <vector>

JNIEXPORT void JNICALL Java_robot_pathfinder_core_path_JNIPath__1construct
        (JNIEnv *env, jobject obj, jobjectArray waypoints, jdouble alpha, jint type) {
    std::vector<rpf::Waypoint> wp(env->GetArrayLength(waypoints));

    for(int i = 0; i < env->GetArrayLength(waypoints); i ++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        wp.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"), rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading")));
    }
    
    rpf::set_obj_ptr(env, obj, rpf::Path::construct_path(wp, alpha, static_cast<rpf::PathType>(type)));
}
