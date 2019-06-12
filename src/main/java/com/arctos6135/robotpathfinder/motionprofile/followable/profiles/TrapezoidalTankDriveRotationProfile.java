package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.DynamicDualMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.TankDriveFollowableRotationMotionProfile;

public class TrapezoidalTankDriveRotationProfile
        extends TankDriveFollowableRotationMotionProfile<DynamicDualMotionProfile<TrapezoidalMotionProfile>>
        implements DynamicFollowable<TankDriveMoment> {

    public TrapezoidalTankDriveRotationProfile(RobotSpecs specs, double angle) {
        baseWidth = specs.getBaseWidth();
        profile = new DynamicDualMotionProfile<TrapezoidalMotionProfile>(
                // As there are two wheels turning, each only need to go half the distance
                // To put it another way, the radius is now half of the base width instead of
                // the full base width
                // Negate the final distance for left because it turns in reverse
                new TrapezoidalMotionProfile(specs, -angle * baseWidth / 2),
                new TrapezoidalMotionProfile(specs, angle * baseWidth / 2));
    }

    public TrapezoidalTankDriveRotationProfile(RobotSpecs specs, double angle, double initialFacing) {
        this(specs, angle);
        this.initialFacing = initialFacing;
    }

    @Override
    public void update(TankDriveMoment m) {
        profile.updateLeft(m.getTime(), m.getLeftPosition(), m.getLeftVelocity(), m.getLeftAcceleration());
        profile.updateRight(m.getTime(), m.getRightPosition(), m.getRightVelocity(), m.getRightAcceleration());
    }
}
