package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.BasicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;

public abstract class BasicFollowableMotionProfile implements BasicFollowable {

    protected MotionProfile profile;

    protected double initialFacing = Math.PI / 2;

    @Override
    public double totalTime() {
        return profile.totalTime();
    }

    @Override
    public BasicMoment get(double t) {
        double d = profile.distance(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        boolean backwards = profile.isReversed();

        return new BasicMoment(d, v, a, backwards ? -initialFacing : initialFacing, t, 
                initialFacing, backwards);
    }
}
