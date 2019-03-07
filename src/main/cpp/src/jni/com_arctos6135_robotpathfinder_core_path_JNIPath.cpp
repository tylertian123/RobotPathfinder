#include "jni/com_arctos6135_robotpathfinder_core_path_JNIPath.h"
#include "paths.h"
#include "jni/jniutil.h"
#include <vector>

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1construct
        (JNIEnv *env, jobject obj, jobjectArray waypoints, jdouble alpha, jint type) {
    std::vector<rpf::Waypoint> wp;

    for(int i = 0; i < env->GetArrayLength(waypoints); i ++) {
        auto waypoint = env->GetObjectArrayElement(waypoints, i);
        wp.push_back(rpf::Waypoint(rpf::get_field<double>(env, waypoint, "x"), rpf::get_field<double>(env, waypoint, "y"),
                rpf::get_field<double>(env, waypoint, "heading")));
    }
    
    rpf::set_obj_ptr(env, obj, rpf::construct_path(wp, alpha, static_cast<rpf::PathType>(type)));
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1destroy(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj);
    if(ptr) {
        delete ptr;
        rpf::set_obj_ptr<rpf::Path>(env, obj, nullptr);
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1setBaseRadius(JNIEnv *env, jobject obj, jdouble radius) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_base(radius);
}
JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1setBackwards(JNIEnv *env, jobject obj, jboolean backwards) {
    rpf::get_obj_ptr<rpf::Path>(env, obj)->set_backwards(backwards);
}

JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath_at(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->at(t);
    
    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath_derivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->deriv_at(t);

    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath_secondDerivAt(JNIEnv *env, jobject obj, jdouble t) {
    auto v = rpf::get_obj_ptr<rpf::Path>(env, obj)->second_deriv_at(t);

    jclass clazz = env->FindClass("com/arctos6135/robotpathfinder/math/Vec2D");
    jmethodID mid = env->GetMethodID(clazz, "<init>", "(DD)V");
    jobject vec = env->NewObject(clazz, mid, v.x, v.y);
    return vec;
}
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath_wheelsAt(JNIEnv *env, jobject obj, jdouble t) {
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

JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1computeLen(JNIEnv *env, jobject obj, int points) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->compute_len(points);
}
JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1s2T(JNIEnv *env, jobject obj, jdouble s) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->s2t(s);
}
JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1t2S(JNIEnv *env, jobject obj, jdouble t) {
    return rpf::get_obj_ptr<rpf::Path>(env, obj)->t2s(t);
}

JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1mirrorLeftRight(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->mirror_lr();
    return reinterpret_cast<jlong>(ptr);
}
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1mirrorFrontBack(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->mirror_fb();
    return reinterpret_cast<jlong>(ptr);
}
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_path_JNIPath__1retrace(JNIEnv *env, jobject obj) {
    auto ptr = rpf::get_obj_ptr<rpf::Path>(env, obj)->retrace();
    return reinterpret_cast<jlong>(ptr);
}
