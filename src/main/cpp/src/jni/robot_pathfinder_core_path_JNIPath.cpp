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
    
    rpf::set_obj_ptr(env, obj, rpf::construct_path(wp, alpha, static_cast<rpf::PathType>(type)));
}

JNIEXPORT void JNICALL Java_robot_pathfinder_core_path_JNIPath__1destroy(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj);
    if(ptr) {
        delete ptr;
        rpf::set_obj_ptr<rpf::Path>(env, obj, nullptr);
    }
}

JNIEXPORT void JNICALL Java_robot_pathfinder_core_path_JNIPath__1setBaseRadius(JNIEnv *env, jobject obj, jdouble radius) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_base(radius);
}
JNIEXPORT void JNICALL Java_robot_pathfinder_core_path_JNIPath__1setBackwards(JNIEnv *env, jobject obj, jboolean backwards) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_backwards(backwards);
}

JNIEXPORT jobject JNICALL Java_robot_pathfinder_core_path_JNIPath__1at(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->at(t);
    
    jclass clazz = env->FindClass("robot/pathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.get_x(), v.get_y());
    return vec;
}
JNIEXPORT jobject JNICALL Java_robot_pathfinder_core_path_JNIPath__1derivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->deriv_at(t);

    jclass clazz = env->FindClass("robot/pathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.get_x(), v.get_y());
    return vec;
}
JNIEXPORT jobject JNICALL Java_robot_pathfinder_core_path_JNIPath__1secondDerivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->second_deriv_at(t);

    jclass clazz = env->FindClass("robot/pathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.get_x(), v.get_y());
    return vec;
}
