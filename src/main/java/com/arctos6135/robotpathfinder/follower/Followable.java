package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

/**
 * This interface defines the basic requirements for a trajectory or motion
 * profile in order to be used by followers in the
 * {@link com.arctos6135.robotpathfinder.follower} package.
 * <p>
 * In order to be considered "followable", a class must provide at least two
 * methods, {@link #get(double)} and {@link #totalTime()}. {@link #get(double)}
 * should return a subclass of {@link Moment} for any time greater than 0 and
 * less than the result of {@link #totalTime()}. This moment contains
 * information about the robot at the specific point in time and is used by a
 * follower. For more information about moments, see the class Javadoc for
 * {@link Moment}.
 * </p>
 * 
 * @author Tyler Tian
 * @param <T> The type of moment used by this {@link Followable}; must be a
 *            subclass of {@link Moment}
 * @see DynamicFollowable
 * @since 3.0.0
 */
public interface Followable<T extends Moment> {
    /**
     * Retrieves the moment associated with the specified time for this
     * {@link Followable}.
     * <p>
     * For more information, see the class Javadoc for this class and
     * {@link Moment}.
     * <p>
     * 
     * @param t The time
     * @return The moment object associated with the specified time
     */
    public T get(double t);

    /**
     * Retrieves the total time it requires to follow this {@link Followable}.
     * <p>
     * For more information, see the class Javadoc for this class.
     * <p>
     * 
     * @return The total time it takes to follow this {@link Followable}.
     */
    public double totalTime();
}
