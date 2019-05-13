package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;

public abstract class FollowableMotionProfile<T extends Moment> implements Followable<T> {
    
    protected MotionProfile profile;

    protected double initialFacing = Math.PI / 2;

    @Override
    public double totalTime() {
        return profile.totalTime();
    }
}
