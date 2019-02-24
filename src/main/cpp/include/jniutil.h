#pragma once

#include <jni.h>

namespace rpf {
    template <typename T>
    T* get_ptr(JNIEnv *env, jobject obj) {
        jfieldID fid = env->GetFieldID(env->GetObjectClass(obj), "_nativePtr", "J");
        return reinterpret_cast<T*>(env->GetLongField(obj, fid));
    }
}
