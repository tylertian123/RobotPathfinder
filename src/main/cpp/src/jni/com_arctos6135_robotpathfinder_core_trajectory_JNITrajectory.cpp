#include "jni/com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory.h"
#include "jni/jniutil.h"
#include "trajectory/basictrajectory.h"
#include <vector>
#include <memory>
#include <algorithm>

std::vector<std::shared_ptr<rpf::BasicTrajectory>> btinstances;
extern std::vector<std::shared_ptr<rpf::Path>> pinstances;

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

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory__1destroy(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    rpf::set_obj_ptr<rpf::BasicTrajectory>(env, obj, nullptr);
    // Remove an entry from the instances list
    auto it = std::find_if(btinstances.begin(), btinstances.end(), [&](const auto &p){ return p.get() == ptr; });
    
    if(it != btinstances.end()) {
        btinstances.erase(it);
    }
    else {
        jclass exclass = env->FindClass("com/arctos6135/robotpathfinder/core/JNIException");
        env->ThrowNew(exclass, "This instance of BasicTrajectory was not found in the instances list");
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory__1getMoments(JNIEnv *env, jobject obj) {
    auto &moments = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get_moments();
    
    jclass clazz = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(clazz, "momentsCache", "[Lcom/arctos6135/robotpathfinder/core/BasicMoment;");
    jobject objf = env->GetObjectField(obj, fid);
    jobjectArray *arr = reinterpret_cast<jobjectArray *>(&objf);

    jmethodID constructor_mid = env->GetMethodID(clazz, "<init>", "(DDDDDDZ)V");
    
    for(size_t i = 0; i < moments.size(); i ++) {
        jobject m = env->NewObject(clazz, constructor_mid, moments[i].dist, moments[i].vel, moments[i].accel,
                moments[i].heading, moments[i].time, moments[i].init_facing, moments[i].backwards);
        env->SetObjectArrayElement(*arr, i, m);
    }
}

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory__1get(JNIEnv *env, jobject obj, jdouble t) {
    auto m = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get(t);
    jmethodID constructor_mid = env->GetMethodID(env->GetObjectClass(obj), "<init>", "(DDDDDDZ)V");

    return env->NewObject(env->GetObjectClass(obj), constructor_mid, m.dist, m.vel, m.accel, m.heading, m.time, m.init_facing, m.backwards);
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNIBasicTrajectory__1getPath(JNIEnv *env, jobject obj) {
    // Since the shared_ptr that came from get_path is a copy of the trajectory's shared_ptr,
    // the reference counting still works
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get_path();
    // Add to the instances list
    pinstances.push_back(ptr);
    // Return the raw address
    return reinterpret_cast<jlong>(ptr.get());
}
