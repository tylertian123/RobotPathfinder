/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory */

#ifndef _Included_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
#define _Included_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _construct
 * Signature: (DDDZ[Lcom/arctos6135/robotpathfinder/core/JNIWaypoint;DII)V
 */
JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1construct
  (JNIEnv *, jobject, jdouble, jdouble, jdouble, jboolean, jobjectArray, jdouble, jint, jint);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _destroy
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1destroy
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _getMoments
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1getMoments
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _get
 * Signature: (D)Lcom/arctos6135/robotpathfinder/core/trajectory/TankDriveMoment;
 */
JNIEXPORT jobject JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1get
  (JNIEnv *, jobject, jdouble);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _getPath
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1getPath
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    totalTime
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory_totalTime
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _mirrorLeftRight
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1mirrorLeftRight
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _mirrorFrontBack
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1mirrorFrontBack
  (JNIEnv *, jobject);

/*
 * Class:     com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory
 * Method:    _retrace
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_arctos6135_robotpathfinder_core_trajectory_JNITankDriveTrajectory__1retrace
  (JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif
