package robot.pathfinder.core;

import robot.pathfinder.math.Vec2D;

/**
 * <b>TODO: Complete docs</b>
 * 
 * Represents a "waypoint" in a path or trajectory. A waypoint consists of an X value, Y value, and heading
 * (direction the robot is travelling in). All angles are in <em>radians</em>. Used to construct paths and trajectories.
 * This class is immutable.
 * <p>
 * The {@code WaypointEx} adds additional velocity and acceleration information in addition to the {@code Waypoint}'s 
 * position and heading. This allows a path to start as a continuation of some other path.
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
    
    final double velocity, acceleration;

    public WaypointEx(double x, double y, double heading, double velocity, double acceleration) {
        super(x, y, heading);
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public WaypointEx(Vec2D location, double heading, double velocity, double acceleration) {
        super(location, heading);
        this.velocity = velocity;
        this.acceleration = acceleration;

    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }
}
