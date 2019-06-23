package com.arctos6135.robotpathfinder.motionprofile;

/**
 * This interface defines the basic requirements for a motion profile.
 * <p>
 * In RobotPathfinder, motion profiles are similar to trajectories; they both
 * provide the position, velocity and acceleration of the robot for any given
 * time. However, motion profiles are much more simple and typically do not
 * allow for turning. They also do not implement
 * {@link com.arctos6135.robotpathfinder.follower.Followable Followable} by
 * default. To use one, look for classes in the
 * {@link com.arctos6135.robotpathfinder.motionprofile.followable.profiles}
 * package.
 * </p>
 * <p>
 * Because of their simplicity, they're much faster to generate. Therefore, for
 * simple paths they're highly recommended over trajectories.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public interface MotionProfile {

    /**
     * Retrieves the total time it takes to complete this {@link MotionProfile}.
     * 
     * @return The total time
     */
    public double totalTime();

    /**
     * Retrieves the position at the specified time in this {@link MotionProfile}.
     * 
     * @param t The time
     * @return The position at the specified time
     */
    public double position(double t);

    /**
     * Retrieves the velocity at the specified time in this {@link MotionProfile}.
     * 
     * @param t The time
     * @return The velocity at the specified time
     */
    public double velocity(double t);

    /**
     * Retrieves the acceleration at the specified time in this
     * {@link MotionProfile}.
     * 
     * @param t The time
     * @return The acceleration at the specified time
     */
    public double acceleration(double t);

    /**
     * Retrieves whether this {@link MotionProfile} is reversed (driving backwards).
     * 
     * @return Whether this profile is reversed
     */
    public boolean isReversed();
}
