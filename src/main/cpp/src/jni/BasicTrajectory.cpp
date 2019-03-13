#include "jni/com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory.h"
#include "jni/jniutil.h"
#include "trajectory/basictrajectory.h"
#include <vector>
#include <list>
#include <memory>
#include <algorithm>

std::list<std::shared_ptr<rpf::BasicTrajectory>> btinstances;
extern std::list<std::shared_ptr<rpf::Path>> pinstances;

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1construct
        (JNIEnv *env, jobject obj, jdouble maxv, jdouble maxa, jdouble base_width, jboolean is_tank, jobjectArray waypoints, jdouble alpha, jint sample_count, jint type) {
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
    params.sample_count = sample_count;
    params.type = static_cast<rpf::PathType>(type);
    params.alpha = alpha;
    
    try {
        rpf::BasicTrajectory *t = new rpf::BasicTrajectory(specs, params);
        btinstances.push_back(std::shared_ptr<rpf::BasicTrajectory>(t));
        rpf::set_obj_ptr(env, obj, t);
    }
    catch(const std::exception &e) {
        jclass exclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/TrajectoryGenerationException");
        env->ThrowNew(exclass, e.what());
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1destroy(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    rpf::set_obj_ptr<rpf::BasicTrajectory>(env, obj, nullptr);
    // Remove an entry from the instances list
    auto it = std::find_if(btinstances.begin(), btinstances.end(), [&](const auto &p){ return p.get() == ptr; });
    
    if(it != btinstances.end()) {
        btinstances.erase(it);
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getMoments(JNIEnv *env, jobject obj) {
    auto &moments = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get_moments();
    
    jclass clazz = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(clazz, "momentsCache", "[Lcom/arctos6135/robotpathfinder/core/trajectory/BasicMoment;");
    jobject objf = env->GetObjectField(obj, fid);
    jobjectArray *arr = reinterpret_cast<jobjectArray *>(&objf);

    jclass mclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/BasicMoment");
    jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDZ)V");
    
    for(size_t i = 0; i < moments.size(); i ++) {
        jobject m = env->NewObject(mclass, constructor_mid, moments[i].dist, moments[i].vel, moments[i].accel,
                moments[i].heading, moments[i].time, moments[i].init_facing, moments[i].backwards);
        env->SetObjectArrayElement(*arr, i, m);
    }
}

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1get(JNIEnv *env, jobject obj, jdouble t) {
    auto m = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get(t);
    jclass mclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/BasicMoment");
    jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDZ)V");

    return env->NewObject(mclass, constructor_mid, m.dist, m.vel, m.accel, m.heading, m.time, m.init_facing, m.backwards);
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getPath(JNIEnv *env, jobject obj) {
    // Since the shared_ptr that came from get_path is a copy of the trajectory's shared_ptr,
    // the reference counting still works
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get_path();
    // Add to the instances list
    pinstances.push_back(ptr);
    // Return the raw address
    return reinterpret_cast<jlong>(ptr.get());
}

JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory_totalTime(JNIEnv *env, jobject obj) {
    return rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->total_time();
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1mirrorLeftRight(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->mirror_lr();
    btinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1mirrorFrontBack(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->mirror_fb();
    btinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1retrace(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->retrace();
    btinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}

JNIEXPORT jint JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getMomentCount(JNIEnv *env, jobject obj) {
    return rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj)->get_moments().size();
}

