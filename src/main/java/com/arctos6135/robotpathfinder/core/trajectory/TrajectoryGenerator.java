package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.path.PathType;

/**
 * This class provides a number of handy static utility functions for generating trajectories.
 * @author Tyler Tian
 *
 */
final public class TrajectoryGenerator {
	
	private TrajectoryGenerator() {}
	
	/**
	 * Generates a {@link JNIBasicTrajectory} that will drive straight forward for the specified distance.
	 * @param specs The specifications of the robot
	 * @param distance The distance to drive forward for
	 * @return The generated trajectory
	 */
	public static JNIBasicTrajectory generateStraightBasic(RobotSpecs specs, double distance) {
		JNITrajectoryParams params = new JNITrajectoryParams();
		params.isTank = false;
		// Use Bezier type since we only have one segment and it doesn't really matter
		params.pathType = PathType.BEZIER;
		// Since we're going in a straight line, we don't need much precision
		params.segmentCount = 100;
		params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0, 0, Math.PI / 2),
            // Take the absolute value; negative values aren't handled so well
            new JNIWaypoint(0, Math.abs(distance), Math.PI / 2)
		};
		params.alpha = Math.abs(distance) / 2;
		// Now check if our distance is negative, and if yes just reverse the trajectory
		return distance >= 0 ? new JNIBasicTrajectory(specs, params) : new JNIBasicTrajectory(specs, params).mirrorFrontBack();
	}
	/**
	 * Generates a {@link JNITankDriveTrajectory} that will drive straight forward for the specified distance.
	 * @param specs The specifications of the robot
	 * @param distance The distance to drive forward for
	 * @return The generated trajectory
	 */
	public static JNITankDriveTrajectory generateStraightTank(RobotSpecs specs, double distance) {
		JNITrajectoryParams params = new JNITrajectoryParams();
		params.isTank = true;
		params.pathType = PathType.BEZIER;
		params.segmentCount = 100;
		params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0, 0, Math.PI / 2),
            new JNIWaypoint(0, Math.abs(distance), Math.PI / 2)
		};
		params.alpha = Math.abs(distance) / 2;
		
		return distance >= 0 ? new JNITankDriveTrajectory(specs, params) : new JNITankDriveTrajectory(specs, params).mirrorFrontBack();
    }

    private static native JNITankDriveTrajectory _generateRotationTank(double maxV, double maxA, double baseWidth, double angle);
    public static JNITankDriveTrajectory generateRotationTank(RobotSpecs specs, double angle) {
        return _generateRotationTank(specs.getMaxVelocity(), specs.getMaxAcceleration(), specs.getBaseWidth(), angle);
    }
}
