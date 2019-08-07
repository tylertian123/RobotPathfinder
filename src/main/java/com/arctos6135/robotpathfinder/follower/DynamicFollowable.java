package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

/**
 * This interface is an extension of {@link Followable} and defines the basic
 * requirements for a trajectory or motion profile to be considered "dynamically
 * followable".
 * <p>
 * See {@link Followable} for the basic definition of a "followable" object. In
 * addition to the constraints specified by {@link Followable}, a class must
 * also provide an additional method {@link #update(Moment)}. This method takes
 * information about the <em>real life</em> conditions of the robot (represented
 * by a moment of type {@link T}), and updates/re-generates the object to better
 * match those conditions.
 * </p>
 * 
 * @author Tyler Tian
 * @param <T> The type of moment used by this {@link DynamicFollowable}; must be
 *            a subclass of {@link Moment}
 * @see Followable
 * @since 3.0.0
 */
public interface DynamicFollowable<T extends Moment> extends Followable<T> {
    /**
     * This method takes information about the <em>real life</em> conditions of the
     * robot (represented by a moment of type {@link T}), and updates/re-generates
     * the object to better match those conditions.
     * <p>
     * For more information, see the class Javadoc for this class.
     * <p>
     * 
     * @param m The real life conditions of the robot
     */
    public void update(T m);
}
