package com.arctos6135.robotpathfinder.core;

import com.arctos6135.robotpathfinder.math.Vec2D;

/**
 * Represents a "waypoint" in a path or trajectory. A waypoint consists of an X value, Y value, and heading
 * (direction the robot is travelling in). All angles are in <em>radians</em>. Used to construct paths and trajectories.
 * This class is immutable.
 * <p>
 * The {@code WaypointEx} adds additional velocity information in addition to the {@link robot.pathfinder.core.Waypoint Waypoint}'s 
 * position and heading. Since it inherits from a regular {@link robot.pathfinder.core.Waypoint Waypoint},
 * you can use it in the place of one to accomplish things such as creating a trajectory that is a continuation of some other
 * trajectory, or make the robot slow down in certain places along the path.
 * </p>
 * <p>
 * Note that it does not matter what specific unit is used for distance; however, the unit must match with 
 * the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the unit
 * for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
 * On the other hand, the angles must all be in <strong>radians</strong>.
 * </p>
 * @author Tyler Tian
 *
 */
public class WaypointEx extends Waypoint {
    
    final double velocity;

    /**
     * Constructs a new waypoint with the specified X and Y value, heading and velocity.
     * <p>
     * Note that it does not matter what specific unit is used for distance and velocity; however, the unit must 
     * match with the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the 
     * unit for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
     * </p>
     * @param x The X value of this waypoint
     * @param y The Y value of this waypoint
     * @param heading The heading at this waypoint, in <strong>radians</strong>
     * @param velocity The velocity of this waypoint
     */
    public WaypointEx(double x, double y, double heading, double velocity) {
        super(x, y, heading);
        this.velocity = velocity;
    }

    /**
     * Constructs a new waypoint with the specified location, heading and velocity.
     * <p>
     * Note that it does not matter what specific unit is used for distance and velocity; however, the unit must 
     * match with the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the 
     * unit for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
     * </p>
     * @param location The location of the waypoint, represented as a 2D vector
     * @param heading The heading at this waypoint, in <strong>radians</strong>
     * @param velocity The velocity of this waypoint
     */
    public WaypointEx(Vec2D location, double heading, double velocity) {
        super(location, heading);
        this.velocity = velocity;
    }

    /**
     * Retrieves the velocity of this waypoint.
     * 
     * @return The velocity of this waypoint
     */
    public double getVelocity() {
        return velocity;
    }
}
