package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.follower.Followable;

public abstract class FollowableMotionProfile<T extends Moment> implements Followable<T> {
    protected double initialFacing = Math.PI / 2;
}
