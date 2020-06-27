package com.arctos6135.robotpathfinder.core;

import java.util.Objects;

/**
 * Represents a point that the robot must pass through in a path or trajectory.
 * <p>
 * A waypoint consists of an X value, Y value, and heading (direction the robot
 * is travelling in). Used to construct paths and trajectories. Waypoints are
 * immutable.
 * </p>
 * <p>
 * The angle used for heading is in <em>radians</em>, following the unit circle.
 * Thus, 0 represents right, &pi;/2 represents up, and so on.
 * </p>
 * <p>
 * Note that it does not matter what specific unit is used for distance;
 * however, the unit must match with the units in the {@link RobotSpecs} object
 * used to construct the trajectory. For example, if the unit for max velocity
 * in the {@link RobotSpecs} object was m/s, the unit used for distance must be
 * m.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class Waypoint {

    protected double x;
    protected double y;
    protected double heading;
    protected double velocity = Double.NaN;

    @Override
    public boolean equals(Object o) {
        if (o == this)
            return true;
        if (!(o instanceof Waypoint)) {
            return false;
        }
        Waypoint waypoint = (Waypoint) o;
        return x == waypoint.x && y == waypoint.y && heading == waypoint.heading && velocity == waypoint.velocity;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, heading, velocity);
    }

    @Override
    public String toString() {
        return "{" + " x='" + getX() + "'" + ", y='" + getY() + "'" + ", heading='" + getHeading() + "'"
                + ", velocity='" + getVelocity() + "'" + "}";
    }

    /**
     * Creates a new {@link Waypoint} with all fields set to 0. The velocity is
     * unconstrained.
     */
    public Waypoint() {
    }

    /**
     * Creates a new {@link Waypoint} with the specified parameters. The velocity is
     * unconstrained.
     * 
     * @param x       The x coordinate of the waypoint
     * @param y       The y coordinate of the waypoint
     * @param heading The heading of the robot at the waypoint, in radians
     */
    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Creates a new {@link Waypoint} with the specified parameters.
     * 
     * @param x        The x coordinate of the waypoint
     * @param y        The y coordinate of the waypoint
     * @param heading  The heading of the robot at the waypoint, in radians
     * @param velocity the velocity of the robot at the waypoint
     */
    public Waypoint(double x, double y, double heading, double velocity) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocity = velocity;
    }

    /**
     * Retrieves the x coordinate of this waypoint.
     * 
     * @return The x coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Retrieves the y coordinate of this waypoint.
     * 
     * @return The y coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * Retrieves the heading of this waypoint.
     * 
     * @return The heading
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Retrieves the velocity of the robot at this waypoint.
     * <p>
     * By default, this value is set to {@code NaN} to signify that the velocity is
     * unconstrained.
     * </p>
     * 
     * @return The velocity
     */
    public double getVelocity() {
        return velocity;
    }
}
