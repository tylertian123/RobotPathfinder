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
 * <p>
 * Note that calling the method {@link #update(Moment)} will modify the
 * {@link DynamicFollowable} itself. If the dynamic followable object needs to
 * be reused, the {@link #copy()} method can be used to create an identical
 * copy, while maintaining the original object.
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
     * Note that calling this method will modify the {@link DynamicFollowable}
     * itself. If the dynamic followable object needs to be reused, the
     * {@link #copy()} method can be used to create an identical copy, while
     * maintaining the original object.
     * </p>
     * <p>
     * For more information, see the class Javadoc for this class.
     * <p>
     * 
     * @param m The real life conditions of the robot
     */
    public void update(T m);

    /**
     * Creates an identical deep copy of this {@link DynamicFollowable} object.
     * 
     * @return An identical copy of this object
     */
    public DynamicFollowable<T> copy();
}
