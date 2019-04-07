package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;

public class TrapezoidalBasicProfile extends BasicFollowableMotionProfile {
    
    public TrapezoidalBasicProfile(RobotSpecs specs, double distance) {
        profile = new TrapezoidalMotionProfile(specs, distance);
    }

    public TrapezoidalBasicProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }
}
