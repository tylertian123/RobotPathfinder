#include "jni/com_arctos6135_robotpathfinder_core_lifecycle_JNIObjectReference.h"
#include "trajectories.h"
#include <memory>
#include <list>
#include <algorithm>

extern std::list<std::shared_ptr<rpf::Path>> pinstances;
extern std::list<std::shared_ptr<rpf::BasicTrajectory>> btinstances;
extern std::list<std::shared_ptr<rpf::TankDriveTrajectory>> ttinstances;

template <typename T>
bool remove(std::list<std::shared_ptr<T>> &instances, T *ptr) {
    auto it = std::find_if(instances.begin(), instances.end(), [&](const auto &p){ return p.get() == ptr; });
    if(it != instances.end()) {
        instances.erase(it);
        return true;
    }
    else {
        return false;
    }
}

JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_lifecycle_JNIObjectReference__1freeObject(JNIEnv *env, jclass clazz, jlong ptr) {
    if(remove(pinstances, reinterpret_cast<rpf::Path *>(ptr))) {
        return;
    }
    if(remove(btinstances, reinterpret_cast<rpf::BasicTrajectory *>(ptr))) {
        return;
    }
    if(remove(ttinstances, reinterpret_cast<rpf::TankDriveTrajectory *>(ptr))) {
        return;
    }
}

