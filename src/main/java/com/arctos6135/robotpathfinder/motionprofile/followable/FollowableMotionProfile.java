package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.follower.Followable;

/**
 * This abstract class is the parent of all classes in this package.
 * 
 * @author Tyler Tian
 * @param <T> The type of moment used by the {@link Followable}; must be a
 *            subclass of {@link Moment}
 * @since 3.0.0
 */
public abstract class FollowableMotionProfile<T extends Moment> implements Followable<T> {
    protected double initialFacing = Math.PI / 2;
}
