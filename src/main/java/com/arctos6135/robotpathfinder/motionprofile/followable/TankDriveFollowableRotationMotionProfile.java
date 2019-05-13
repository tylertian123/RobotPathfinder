package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.MathUtils;

public abstract class TankDriveFollowableRotationMotionProfile extends FollowableMotionProfile<TankDriveMoment>
        implements Followable<TankDriveMoment> {

    protected double baseWidth;

    @Override
    public TankDriveMoment get(double t) {
        double d = profile.position(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        // Positive angles, which make the distance positive, are left turns
        double currentAngle = MathUtils.restrictAngle(d / (baseWidth / 2) + initialFacing);

        return new TankDriveMoment(-d, d, -v, v, -a, a, currentAngle, t, initialFacing, false);
    }
}
