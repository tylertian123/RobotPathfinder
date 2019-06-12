package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.motionprofile.DualMotionProfile;

public abstract class TankDriveFollowableRotationMotionProfile<T extends DualMotionProfile<?>> extends FollowableMotionProfile<TankDriveMoment> {

    protected T profile;

    protected double baseWidth;

    protected double initialFacing = Math.PI / 2;

    @Override
    public double totalTime() {
        return profile.totalTime();
    }

    @Override
    public TankDriveMoment get(double t) {
        double ld = profile.leftPosition(t);
        double rd = profile.rightPosition(t);

        // The overall angle rotated is equivalent to the angle caused by the right
        // wheel moving and the angle caused by the left wheel moving combined.
        // Since the angle is in radians, simply divide the arc length by the radius to
        // get the angle.
        // The left distance is negative since it moving forward will actually turn the
        // robot clockwise.
        double currentAngle = rd / baseWidth - ld / baseWidth;
        return new TankDriveMoment(ld, rd, profile.leftVelocity(t), profile.rightVelocity(t),
                profile.leftAcceleration(t), profile.rightAcceleration(t), currentAngle, t, initialFacing, false);
    }
}
