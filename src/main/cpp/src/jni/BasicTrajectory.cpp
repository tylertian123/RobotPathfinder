#include "jni/com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory.h"
#include "jni/jniutil.h"
#include "trajectory/basictrajectory.h"
#include "jni/instlists.h"
#include <vector>
#include <algorithm>

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1construct
        (JNIEnv *env, jobject obj, jdouble maxv, jdouble maxa, jdouble base_width, jboolean is_tank, jobjectArray waypoints, jdouble alpha, jint sample_count, jint type) {
    rpf::TrajectoryParams params;
    params.waypoints.reserve(env->GetArrayLength(waypoints));
    // Translate the waypoints into C++ ones
    for(int i = 0; i < env->GetArrayLength(waypoints); i ++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        params.waypoints.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"), rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading"), rpf::get_field<double>(env, waypoint, "velocity")));
    }

    rpf::RobotSpecs specs(maxv, maxa, base_width);
    params.is_tank = is_tank;
    params.sample_count = sample_count;
    params.type = static_cast<rpf::PathType>(type);
    params.alpha = alpha;
    
    try {
        rpf::BasicTrajectory *t = new rpf::BasicTrajectory(specs, params);
        {
            // Acquire lock to btinstances mutex
            std::lock_guard<std::mutex> lock(btinstances_mutex);
            btinstances.push_back(std::shared_ptr<rpf::BasicTrajectory>(t));
        }
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
    rpf::remove_instance(btinstances, btinstances_mutex, ptr);
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getMoments(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
    }
    else {
        auto &moments = ptr->get_moments();
        
        jclass clazz = env->GetObjectClass(obj);
        jfieldID fid = env->GetFieldID(clazz, "momentsCache", "[Lcom/arctos6135/robotpathfinder/core/trajectory/BasicMoment;");
        jobject objf = env->GetObjectField(obj, fid);
        jobjectArray *arr = reinterpret_cast<jobjectArray *>(&objf);

        jclass mclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/BasicMoment");
        jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDZ)V");
        
        for(size_t i = 0; i < moments.size(); i ++) {
            jobject m = env->NewObject(mclass, constructor_mid, moments[i].pos, moments[i].vel, moments[i].accel,
                    moments[i].heading, moments[i].time, moments[i].init_facing, moments[i].backwards);
            env->SetObjectArrayElement(*arr, i, m);
        }
    }
}

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1get(JNIEnv *env, jobject obj, jdouble t) {
    auto ptr = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return NULL;
    }
    else {
        auto m = ptr->get(t);
        jclass mclass = env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/BasicMoment");
        jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDZ)V");

        return env->NewObject(mclass, constructor_mid, m.pos, m.vel, m.accel, m.heading, m.time, m.init_facing, m.backwards);
    }
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getPath(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        // Since the shared_ptr that came from get_path is a copy of the trajectory's shared_ptr,
        // the reference counting still works
        auto ptr = p->get_path();
        // Add to the instances list
        pinstances.push_back(ptr);
        // Return the raw address
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory_totalTime(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        return p->total_time();
    }
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1mirrorLeftRight(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->mirror_lr();
        {
            // Acquire lock to btinstances mutex
            std::lock_guard<std::mutex> lock(btinstances_mutex);
            btinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1mirrorFrontBack(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->mirror_fb();
        {
            // Acquire lock to btinstances mutex
            std::lock_guard<std::mutex> lock(btinstances_mutex);
            btinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1retrace(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->retrace();
        {
            // Acquire lock to btinstances mutex
            std::lock_guard<std::mutex> lock(btinstances_mutex);
            btinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jint JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_BasicTrajectory__1getMomentCount(JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::BasicTrajectory>(env, obj);
    if(!rpf::check_instance(btinstances, btinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        return p->get_moments().size();
    }
}

