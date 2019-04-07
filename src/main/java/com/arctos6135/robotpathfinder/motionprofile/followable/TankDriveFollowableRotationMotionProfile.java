package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.TankDriveFollowable;

public abstract class TankDriveFollowableRotationMotionProfile extends FollowableMotionProfile
        implements TankDriveFollowable {
    
    protected double baseWidth;

    @Override
    public TankDriveMoment get(double t) {
        double d = profile.position(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        // Positive angles, which make the distance positive, are left turns
        double currentAngle = d / baseWidth + initialFacing;

        return new TankDriveMoment(-d, d, -v, v, -a, a, currentAngle, t, initialFacing, false);
    }
}
