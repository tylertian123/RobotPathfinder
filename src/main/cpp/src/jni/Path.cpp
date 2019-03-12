#include "jni/com_arctos6135_robotpathfinder_core_path_Path.h"
#include "paths.h"
#include "jni/jniutil.h"
#include <vector>
#include <list>
#include <memory>
#include <algorithm>

// This is a list of all the existing instances of Path (Java side)
// With each instance created an entry will be added
// With each instance destroyed an entry will be deleted
// This ensures that when there are no more Java instances of a Path object, the C++ object is also deleted
std::list<std::shared_ptr<rpf::Path>> pinstances;

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1construct
        (JNIEnv *env, jobject obj, jobjectArray waypoints, jdouble alpha, jint type) {
    std::vector<rpf::Waypoint> wp;
    wp.reserve(env->GetArrayLength(waypoints));
    // Translate the waypoints into C++ ones
    for(int i = 0; i < env->GetArrayLength(waypoints); i ++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        wp.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"), rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading")));
    }
    
    rpf::Path *path = new rpf::Path(wp, alpha, static_cast<rpf::PathType>(type));
    // Add the newly created path to the instances list
    pinstances.push_back(std::shared_ptr<rpf::Path>(path));
    rpf::set_obj_ptr(env, obj, path);
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1destroy(JNIEnv *env, jobject obj) {
    // Retrieve the pointer and set it to null
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj);
    rpf::set_obj_ptr<rpf::Path>(env, obj, nullptr);
    // Remove an entry from the instances list
    auto it = std::find_if(pinstances.begin(), pinstances.end(), [&](const auto &p){ return p.get() == ptr; });
    
    if(it != pinstances.end()) {
        pinstances.erase(it);
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1setBaseRadius(JNIEnv *env, jobject obj, jdouble radius) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_base(radius);
}
JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1setBackwards(JNIEnv *env, jobject obj, jboolean backwards) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_backwards(backwards);
}

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path_at(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->at(t);
    
    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path_derivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->deriv_at(t);

    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path_secondDerivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->second_deriv_at(t);

    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path_wheelsAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->wheels_at(t);

    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/util/Pair");
    jmethodID pcmid = env->GetMethodID(clazz, "<init>", "(Ljava/lang/Object;Ljava/lang/Object;)V");
    jclass clazz2 = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID vcmid = env->GetMethodID(clazz2, "<init>", "(DD)V");
    
    jobject left = env->NewObject(clazz2, vcmid, v.first.x, v.first.y);
    jobject right = env->NewObject(clazz2, vcmid, v.second.x, v.second.y);
    jobject p = env->NewObject(clazz, pcmid, left, right);
    return p;
}

JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1computeLen(JNIEnv *env, jobject obj, int points) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->compute_len(points);
}
JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1s2T(JNIEnv *env, jobject obj, jdouble s) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->s2t(s);
}
JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1t2S(JNIEnv *env, jobject obj, jdouble t) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->t2s(t);
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1mirrorLeftRight(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->mirror_lr();
    pinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1mirrorFrontBack(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->mirror_fb();
    pinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1retrace(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->retrace();
    pinstances.push_back(ptr);
    return reinterpret_cast<jlong>(ptr.get());
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_Path__1updateWaypoints(JNIEnv *env, jobject obj) {
    auto &wp = rpf::get_obj_ptr<rpf::Path>(env, obj)->get_waypoints();
    jclass clazz = env->GetObjectClass(obj);
    jclass wpclass = env->FindClass("com/arctos6135/robotpathfinder/core/Waypoint");
    jfieldID fid = env->GetFieldID(clazz, "waypoints", "[Lcom/arctos6135/robotpathfinder/core/Waypoint;");
    jobject objf = env->GetObjectField(obj, fid);
    jobjectArray *arr = reinterpret_cast<jobjectArray *>(&objf);

    jmethodID constructor_mid = env->GetMethodID(wpclass, "<init>", "(DDDD)V");
    for(size_t i = 0; i < wp.size(); i ++) {
        jobject w = env->NewObject(wpclass, constructor_mid, wp[i].x, wp[i].y, wp[i].heading, wp[i].velocity);
        env->SetObjectArrayElement(*arr, i, w);
    }
}
