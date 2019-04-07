package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;

public class TrapezoidalTankDriveProfile extends TankDriveFollowableMotionProfile {
    
    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance) {
        profile = new TrapezoidalMotionProfile(specs, distance);
    }

    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }
}
