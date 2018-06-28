/**
 * This package contains all the different types of trajectories, as well as their customized moment objects, of
 * RobotPathfinder.
 * <p>
 * A trajectory not only defines the points that a robot will path through, it also provides information about the
 * velocity, acceleration and direction. Most importantly, for any given time, it can provide information about
 * the robot's whereabouts and other information. Note that unlike paths and spline segments, there is no common
 * ancestor for all the trajectory types, because different types of trajectories are suited to different types
 * of robots, and thus have very different interfaces.
 * </p>
 */
package robot.pathfinder.core.trajectory;