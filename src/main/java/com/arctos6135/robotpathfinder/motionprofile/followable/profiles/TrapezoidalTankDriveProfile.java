package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.DynamicDualMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.TankDriveFollowableMotionProfile;

public class TrapezoidalTankDriveProfile
        extends TankDriveFollowableMotionProfile<DynamicDualMotionProfile<TrapezoidalMotionProfile>>
        implements DynamicFollowable<TankDriveMoment> {

    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance) {
        profile = new DynamicDualMotionProfile<TrapezoidalMotionProfile>(new TrapezoidalMotionProfile(specs, distance),
                new TrapezoidalMotionProfile(specs, distance));
    }

    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }

    @Override
    public void update(TankDriveMoment m) {
        profile.updateLeft(m.getTime(), m.getLeftPosition(), m.getLeftVelocity(), m.getLeftAcceleration());
        profile.updateRight(m.getTime(), m.getRightPosition(), m.getRightVelocity(), m.getRightAcceleration());
    }
}
