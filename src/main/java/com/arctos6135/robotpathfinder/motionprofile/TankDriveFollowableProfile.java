package com.arctos6135.robotpathfinder.motionprofile;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.TankDriveFollowable;

public abstract class TankDriveFollowableProfile implements TankDriveFollowable {

    protected MotionProfile profile;

    protected double initialFacing = Math.PI / 2;

    @Override
    public double totalTime() {
        return profile.totalTime();
    }

    @Override
    public TankDriveMoment get(double t) {
        double d = profile.distance(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        boolean backwards = false;
        if (profile instanceof TrapezoidalMotionProfile) {
            backwards = ((TrapezoidalMotionProfile) profile).reverse;
        }

        return new TankDriveMoment(d, d, v, v, a, a, backwards ? -initialFacing : initialFacing, t, 
                initialFacing, backwards);
    }
}
