#pragma once

#include <jni.h>

namespace rpf {
    template <typename T>
    T* get_obj_ptr(JNIEnv *env, jobject obj) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), "_nativePtr", "J");
        return reinterpret_cast<T*>(env->GetLongField(obj, fid));
    }
    template <typename T>
    void set_obj_ptr(JNIEnv *env, jobject obj, T* ptr) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), "_nativePtr", "J");
        env->SetLongField(obj, fid, reinterpret_cast<jlong>(ptr));
    }
    
    template <typename T>
    T get_field(JNIEnv *env, jobject obj, const char *fname);
    template <>
    jboolean get_field<jboolean>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "Z");
        return env->GetBooleanField(obj, fid);
    }
    template <>
    jbyte get_field<jbyte>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "B");
        return env->GetByteField(obj, fid);
    }
    template <>
    jchar get_field<jchar>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "C");
        return env->GetCharField(obj, fid);
    }
    template <>
    jshort get_field<jshort>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "S");
    }
    template <>
    jint get_field<jint>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "I");
        return env->GetIntField(obj, fid);
    }
    template <>
    jlong get_field<jlong>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "J");
        return env->GetLongField(obj, fid);
    }
    template <>
    jfloat get_field<jfloat>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "F");
        return env->GetFloatField(obj, fid);
    }
    template <>
    jdouble get_field<jdouble>(JNIEnv *env, jobject obj, const char *fname) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), fname, "D");
        return env->GetDoubleField(obj, fid);
    }
}
