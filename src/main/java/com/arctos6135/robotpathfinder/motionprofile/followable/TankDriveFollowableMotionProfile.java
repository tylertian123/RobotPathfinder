package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;

public abstract class TankDriveFollowableMotionProfile extends FollowableMotionProfile<TankDriveMoment>
        implements Followable<TankDriveMoment> {

    @Override
    public TankDriveMoment get(double t) {
        double d = profile.position(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        boolean backwards = profile.isReversed();

        return new TankDriveMoment(d, d, v, v, a, a, backwards ? -initialFacing : initialFacing, t, initialFacing,
                backwards);
    }
}
