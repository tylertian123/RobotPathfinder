/**
 * This package contains all the motion profiles of RobotPathfinder.
 * <p>
 * In RobotPathfinder, motion profiles are similar to trajectories; they both
 * provide the position, velocity and acceleration of the robot for any given
 * time. However, motion profiles are much more simple and typically do not
 * allow for turning. They also do not implement
 * {@link com.arctos6135.robotpathfinder.follower.Followable Followable} by
 * default. To use them, look for classes in the
 * {@link com.arctos6135.robotpathfinder.motionprofile.followable.profiles}
 * package.
 * </p>
 * <p>
 * Because of their simplicity, they're much faster to generate. Therefore, for
 * simple paths they're highly recommended over trajectories.
 * </p>
 */
package com.arctos6135.robotpathfinder.motionprofile;
