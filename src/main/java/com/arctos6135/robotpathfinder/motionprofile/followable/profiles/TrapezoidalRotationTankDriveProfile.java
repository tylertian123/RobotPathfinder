package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.TankDriveFollowableRotationMotionProfile;

public class TrapezoidalRotationTankDriveProfile extends TankDriveFollowableRotationMotionProfile {

    public TrapezoidalRotationTankDriveProfile(RobotSpecs specs, double angle) {
        baseWidth = specs.getBaseWidth();
        profile = new TrapezoidalMotionProfile(specs, angle * baseWidth / 2);
    }

    public TrapezoidalRotationTankDriveProfile(RobotSpecs specs, double angle, double initialFacing) {
        this(specs, angle);
        this.initialFacing = initialFacing;
    }
}
