package robot.pathfinder.core;

import robot.pathfinder.math.Vec2D;

/**
 * Represents a "waypoint" in a path or trajectory. A waypoint consists of an X value, Y value, and heading
 * (direction the robot is facing). All angles are in <em>radians</em>. Used to construct paths and trajectories.
 * <p>
 * Note that it does not matter what specific unit is used for distance; however, the unit must match with 
 * the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the unit
 * for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
 * On the other hand, the angles must all be in <strong>radians</strong>.
 * </p>
 * @author Tyler Tian
 *
 */
public class Waypoint {
	double x, y, heading;
	
	/**
	 * Constructs a new waypoint with the specified X and Y value and heading.
	 * <p>
	 * Note that it does not matter what specific unit is used for distance; however, the unit must match with 
	 * the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the unit
	 * for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
	 * </p>
	 * @param x The X value of this waypoint
	 * @param y The Y value of this waypoint
	 * @param heading The heading at this waypoint, in <strong>radians</strong>
	 */
	public Waypoint(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	/**
	 * Constructs a new waypoint at the specified location with the specified heading.
	 * <p>
	 * Note that it does not matter what specific unit is used for distance; however, the unit must match with 
	 * the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the unit
	 * for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
	 * </p>
	 * @param location The location of the waypoint, represented as a 2D vector
	 * @param heading The direction the robot is heading at this waypoint, in <strong>radians</strong>
	 */
	public Waypoint(Vec2D location, double heading) {
		this.x = location.getX();
		this.y = location.getY();
		this.heading = heading;
	}
	
	/**
	 * Retrieves the X value of this waypoint.
	 * @return The X value of this waypoint
	 */
	public double getX() {
		return x;
	}
	/**
	 * Retrieves the Y value of this waypoint.
	 * @return The Y value of this waypoint
	 */
	public double getY() {
		return y;
	}
	/**
	 * Retrieves the heading of the robot at this waypoint, in radians.
	 * @return The heading in radians
	 */
	public double getHeading() {
		return heading;
	}
	/**
	 * Retrieves the position of this waypoint represented as a {@link Vec2D}.
	 * @return The position of this waypoint
	 */
	public Vec2D asVector() {
		return new Vec2D(x, y);
	}
}
