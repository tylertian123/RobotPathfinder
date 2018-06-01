package robot.pathfinder;

import robot.pathfinder.math.Vec2D;

/**
 * Represents a waypoint in a path or trajectory. A waypoint consists of an X and Y value, along with an 
 * optional direction.
 * @author Tyler Tian
 *
 */
public class Waypoint {
	double x, y, heading;
	
	/**
	 * Constructs a new waypoint with the specified X and Y value and no heading.
	 * @param x - The X value of this waypoint
	 * @param y - The Y value of this waypoint
	 */
	public Waypoint(double x, double y) {
		this.x = x;
		this.y = y;
	}
	/**
	 * Constructs a new waypoint with the specified X and Y value and heading.
	 * @param x - The X value of this waypoint
	 * @param y - The Y value of this waypoint
	 * @param heading - The heading of the robot, in radians.
	 */
	public Waypoint(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	/**
	 * Constructs a new waypoint at the specified location without heading.
	 * @param location The location of the waypoint
	 */
	public Waypoint(Vec2D location) {
		this.x = location.getX();
		this.y = location.getY();
	}
	/**
	 * Constructs a new waypoint at the specified location with the specified heading.
	 * @param location The location of the waypoint
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
	 * Retrieves the heading of the robot, in radians.
	 * @return The heading in radians
	 */
	public double getHeading() {
		return heading;
	}
	/**
	 * Retrieves the position of this waypoint represented as a {@code Vec2D}
	 * @return The position of this waypoint
	 */
	public Vec2D getPosVec() {
		return new Vec2D(x, y);
	}
}
