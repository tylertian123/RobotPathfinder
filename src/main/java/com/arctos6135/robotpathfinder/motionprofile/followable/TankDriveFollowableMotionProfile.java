package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.DualMotionProfile;

public abstract class TankDriveFollowableMotionProfile<T extends DualMotionProfile<?>>
        implements Followable<TankDriveMoment> {

    protected T profile;

    protected double initialFacing = Math.PI / 2;

    @Override
    public double totalTime() {
        return profile.totalTime();
    }

    @Override
    public TankDriveMoment get(double t) {
        boolean backwards = profile.isReversed();
        return new TankDriveMoment(profile.leftPosition(t), profile.rightPosition(t), profile.leftVelocity(t),
                profile.rightVelocity(t), profile.leftAcceleration(t), profile.rightAcceleration(t),
                backwards ? -initialFacing : initialFacing, t, initialFacing, backwards);
    }
}
