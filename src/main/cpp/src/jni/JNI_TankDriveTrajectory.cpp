#include "trajectory/tankdrivetrajectory.h"
#include "jni/com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory.h"
#include "jni/instlists.h"
#include "jni/jniutil.h"
#include "trajectory/basictrajectory.h"
#include <algorithm>
#include <vector>

JNIEXPORT void JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1construct(JNIEnv *env,
        jobject obj, jdouble maxv, jdouble maxa, jdouble base_width, jboolean is_tank,
        jobjectArray waypoints, jdouble alpha, jint sample_count, jint type) {
    std::vector<rpf::Waypoint> wp;
    wp.reserve(env->GetArrayLength(waypoints));
    // Translate the waypoints into C++ ones
    for (int i = 0; i < env->GetArrayLength(waypoints); i++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        wp.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"),
                rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading"),
                rpf::get_field<double>(env, waypoint, "velocity")));
    }

    rpf::RobotSpecs specs(maxv, maxa, base_width);
    rpf::TrajectoryParams params;
    params.waypoints = std::move(wp);
    params.is_tank = is_tank;
    params.sample_count = sample_count;
    params.type = static_cast<rpf::PathType>(type);
    params.alpha = alpha;

    try {
        rpf::BasicTrajectory bt(specs, params);
        auto *t = new rpf::TankDriveTrajectory(bt);
        {
            // Acquire lock
            std::lock_guard<std::mutex> lock(ttinstances_mutex);
            ttinstances.push_back(std::shared_ptr<rpf::TankDriveTrajectory>(t));
        }
        rpf::set_obj_ptr(env, obj, t);
    }
    catch (const std::exception &e) {
        jclass exclass = env->FindClass(
                "com/arctos6135/robotpathfinder/core/trajectory/TrajectoryGenerationException");
        env->ThrowNew(exclass, e.what());
    }
}

JNIEXPORT void JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1destroy(
        JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    rpf::set_obj_ptr<rpf::TankDriveTrajectory>(env, obj, nullptr);
    // Remove an entry from the instances list
    rpf::remove_instance(ttinstances, ttinstances_mutex, ptr);
}

JNIEXPORT void JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1getMoments(
        JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
    }
    else {
        auto &moments = ptr->get_moments();

        jclass clazz = env->GetObjectClass(obj);
        jfieldID fid = env->GetFieldID(clazz, "momentsCache",
                "[Lcom/arctos6135/robotpathfinder/core/trajectory/TankDriveMoment;");
        jobject objf = env->GetObjectField(obj, fid);
        jobjectArray *arr = reinterpret_cast<jobjectArray *>(&objf);

        jclass mclass =
                env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/TankDriveMoment");
        jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDDDDZ)V");

        for (size_t i = 0; i < moments.size(); i++) {
            jobject m = env->NewObject(mclass, constructor_mid, moments[i].l_pos, moments[i].r_pos,
                    moments[i].l_vel, moments[i].r_vel, moments[i].l_accel, moments[i].r_accel,
                    moments[i].heading, moments[i].time, moments[i].init_facing,
                    moments[i].backwards);
            env->SetObjectArrayElement(*arr, i, m);
        }
    }
}

JNIEXPORT jobject JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1get(
        JNIEnv *env, jobject obj, jdouble t) {
    auto ptr = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return NULL;
    }
    else {
        auto m = ptr->get(t);
        jclass mclass =
                env->FindClass("com/arctos6135/robotpathfinder/core/trajectory/TankDriveMoment");
        jmethodID constructor_mid = env->GetMethodID(mclass, "<init>", "(DDDDDDDDDZ)V");

        return env->NewObject(mclass, constructor_mid, m.l_pos, m.r_pos, m.l_vel, m.r_vel,
                m.l_accel, m.r_accel, m.heading, m.time, m.init_facing, m.backwards);
    }
}

JNIEXPORT jlong JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1getPath(
        JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->get_path();
        pinstances.push_back(ptr);
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jdouble JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory_totalTime(
        JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        return ptr->total_time();
    }
}

JNIEXPORT jlong JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1mirrorLeftRight(
        JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->mirror_lr();
        {
            // Acquire lock
            std::lock_guard<std::mutex> lock(ttinstances_mutex);
            ttinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jlong JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1mirrorFrontBack(
        JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->mirror_fb();
        {
            // Acquire lock
            std::lock_guard<std::mutex> lock(ttinstances_mutex);
            ttinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jlong JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1retrace(
        JNIEnv *env, jobject obj) {
    auto p = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, p)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        auto ptr = p->retrace();
        {
            // Acquire lock
            std::lock_guard<std::mutex> lock(ttinstances_mutex);
            ttinstances.push_back(ptr);
        }
        return reinterpret_cast<jlong>(ptr.get());
    }
}

JNIEXPORT jint JNICALL
Java_com_arctos6135_robotpathfinder_core_trajectory_TankDriveTrajectory__1getMomentCount(
        JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::TankDriveTrajectory>(env, obj);
    if (!rpf::check_instance(ttinstances, ttinstances_mutex, ptr)) {
        rpf::throw_IllegalStateException(env, "This object has already been freed");
        return 0;
    }
    else {
        return ptr->get_moments().size();
    }
}
