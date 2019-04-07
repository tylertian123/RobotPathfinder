package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.TankDriveFollowable;

public abstract class TankDriveFollowableRotationMotionProfile extends FollowableMotionProfile
        implements TankDriveFollowable {

    protected double angle;
    protected double baseWidth;

    @Override
    public TankDriveMoment get(double t) {
        double d = profile.distance(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        double currentAngle = (angle > 0 ? d / baseWidth : -d / baseWidth) + initialFacing;
        // Angle > 0 is a left turn
        if(angle > 0) {
            return new TankDriveMoment(-d, d, -v, v, -a, a, currentAngle, t, initialFacing, false);
        }
        else {
            return new TankDriveMoment(d, -d, v, -v, a, -a, currentAngle, t, initialFacing, false);
        }
    }
}
