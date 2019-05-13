package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.Followable;

public abstract class BasicFollowableMotionProfile extends FollowableMotionProfile<BasicMoment>
        implements Followable<BasicMoment> {

    @Override
    public BasicMoment get(double t) {
        double d = profile.position(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        boolean backwards = profile.isReversed();

        return new BasicMoment(d, v, a, backwards ? -initialFacing : initialFacing, t, initialFacing, backwards);
    }
}
