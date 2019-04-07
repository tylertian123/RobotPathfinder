package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;

public class TrapezoidalRotationTankDriveProfile extends TankDriveFollowableRotationMotionProfile {

    public TrapezoidalRotationTankDriveProfile(RobotSpecs specs, double angle) {
        baseWidth = specs.getBaseWidth();
        profile = new TrapezoidalMotionProfile(specs, angle * baseWidth);
    }

    public TrapezoidalRotationTankDriveProfile(RobotSpecs specs, double angle, double initialFacing) {
        this(specs, angle);
        this.initialFacing = initialFacing;
    }
}
