#include "jni/robot_pathfinder_core_trajectory_BasicTrajectoryJNI.h"
#include "jniutil.h"

JNIEXPORT void JNICALL Java_robot_pathfinder_core_trajectory_BasicTrajectoryJNI_construct
        (JNIEnv *env, jobject obj, jdouble max_v, jdouble max_a, jdouble base_width, jobjectArray waypoints, 
        jdouble alpha, jint seg_count, jboolean is_tank, jint path_type) {
    
}
