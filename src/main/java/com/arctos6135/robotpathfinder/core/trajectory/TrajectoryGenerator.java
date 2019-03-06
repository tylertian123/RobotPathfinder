package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.math.Vec2D;

/**
 * This class provides a number of handy static utility functions for generating trajectories.
 * @author Tyler Tian
 *
 */
final public class TrajectoryGenerator {
	
	private TrajectoryGenerator() {}
	
	/**
	 * Generates a {@link BasicTrajectory} that will drive straight forward for the specified distance.
	 * @param specs The specifications of the robot
	 * @param distance The distance to drive forward for
	 * @return The generated trajectory
	 */
	public static BasicTrajectory generateStraightBasic(RobotSpecs specs, double distance) {
		TrajectoryParams params = new TrajectoryParams();
		params.isTank = false;
		// Use Bezier type since we only have one segment and it doesn't really matter
		params.pathType = PathType.BEZIER;
		// Since we're going in a straight line, we don't need much precision
		params.segmentCount = 100;
		params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            // Take the absolute value; negative values aren't handled so well
            new Waypoint(0, Math.abs(distance), Math.PI / 2)
		};
		params.alpha = Math.abs(distance) / 2;
		// Now check if our distance is negative, and if yes just reverse the trajectory
		return distance >= 0 ? new BasicTrajectory(specs, params) : new BasicTrajectory(specs, params).mirrorFrontBack();
	}
	/**
	 * Generates a {@link TankDriveTrajectory} that will drive straight forward for the specified distance.
	 * @param specs The specifications of the robot
	 * @param distance The distance to drive forward for
	 * @return The generated trajectory
	 */
	public static TankDriveTrajectory generateStraightTank(RobotSpecs specs, double distance) {
		TrajectoryParams params = new TrajectoryParams();
		params.isTank = true;
		params.pathType = PathType.BEZIER;
		params.segmentCount = 100;
		params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(0, Math.abs(distance), Math.PI / 2)
		};
		params.alpha = Math.abs(distance) / 2;
		
		return distance >= 0 ? new TankDriveTrajectory(specs, params) : new TankDriveTrajectory(specs, params).mirrorFrontBack();
    }
    
    /**
     * Generates a {@link TankDriveTrajectory} that represents a rotation.
     * <p>
     * <em>Note: The resulting trajectory will not have an underlying path or generation parameters. Therefore, attempting to use
     * methods such as {@link robot.pathfinder.core.trajectory.Trajectory#mirrorLeftRight() mirrorLeftRight()}, 
     * {@link robot.pathfinder.core.trajectory.Trajectory#mirrorFrontBack() mirrorFrontBack()}
     * or {@link robot.pathfinder.core.trajectory.Trajectory#retrace() retrace()} will result in an error.</em>
     * </p>
     * @param specs The specifications of the robot
     * @param angle The angle, in radians, to turn for. Positive angles are counter-clockwise.
     * @return The generated trajectory
     */
    public static TankDriveTrajectory generateRotationTank(RobotSpecs specs, double angle) {
        double distPerRadian = specs.getBaseWidth() / 2;
        
        TankDriveTrajectory raw = generateStraightTank(specs, Math.abs(angle) * distPerRadian);
        raw.path = null;
        raw.params = null;

        for(int i = 0; i < raw.moments.length; i ++) {
            if(angle > 0) {
                // Negate the position, velocity and acceleration of a side
                raw.moments[i].setLeftPosition(-raw.moments[i].getLeftPosition());
                raw.moments[i].setLeftVelocity(-raw.moments[i].getLeftVelocity());
                raw.moments[i].setLeftAcceleration(-raw.moments[i].getLeftAcceleration());
                // Use the unchanged right side position and divide it by the distance per radian to get the angle offset
                // Add it to the initial facing to get the heading
                double heading = MathUtils.restrictAngle(raw.moments[i].getRightPosition() / distPerRadian + raw.moments[i].getInitialFacing());
                raw.moments[i].setHeading(heading);
                raw.headingVectors[i] = new Vec2D(Math.cos(heading), Math.sin(heading));
            }
            else {
                raw.moments[i].setRightPosition(-raw.moments[i].getRightPosition());
                raw.moments[i].setRightVelocity(-raw.moments[i].getRightVelocity());
                raw.moments[i].setRightAcceleration(-raw.moments[i].getRightAcceleration());
                // Negate it this time since turning to the right reduces the angle
                double heading = MathUtils.restrictAngle(-raw.moments[i].getLeftPosition() / distPerRadian + raw.moments[i].getInitialFacing());
                raw.moments[i].setHeading(heading);
                raw.headingVectors[i] = new Vec2D(Math.cos(heading), Math.sin(heading));
            }
        }

        return raw;
    }
}
