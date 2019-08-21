#pragma once

#include <algorithm>
#include <jni.h>
#include <list>
#include <memory>
#include <mutex>

namespace rpf {
    template <typename T>
    T *get_obj_ptr(JNIEnv *env, jobject obj) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), "_nativePtr", "J");
        return reinterpret_cast<T *>(env->GetLongField(obj, fid));
    }
    template <typename T>
    void set_obj_ptr(JNIEnv *env, jobject obj, T *ptr) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), "_nativePtr", "J");
        env->SetLongField(obj, fid, reinterpret_cast<jlong>(ptr));
    }

    template <typename T>
    T get_field(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jboolean get_field<jboolean>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jbyte get_field<jbyte>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jchar get_field<jchar>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jshort get_field<jshort>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jint get_field<jint>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jlong get_field<jlong>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jfloat get_field<jfloat>(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jdouble get_field<jdouble>(JNIEnv *env, jobject obj, const char *fname);

    template <typename T>
    bool remove_instance(
            std::list<std::shared_ptr<T>> &instances, std::mutex &instances_mutex, T *ptr) {
        // Acquire lock to the mutex
        std::lock_guard<std::mutex> lock(instances_mutex);
        auto it = std::find_if(
                instances.begin(), instances.end(), [&](const auto &p) { return p.get() == ptr; });
        if (it != instances.end()) {
            instances.erase(it);
            return true;
        }
        else {
            return false;
        }
    }
    template <typename T>
    bool check_instance(
            std::list<std::shared_ptr<T>> &instances, std::mutex &instances_mutex, T *ptr) {
        // Acquire lock to the mutex
        std::lock_guard<std::mutex> lock(instances_mutex);
        auto it = std::find_if(
                instances.begin(), instances.end(), [&](const auto &p) { return p.get() == ptr; });
        return it != instances.end();
    }

    void throw_IllegalStateException(JNIEnv *, const char *);
} // namespace rpf
