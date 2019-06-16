package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.BasicFollowableMotionProfile;

public class TrapezoidalBasicProfile extends BasicFollowableMotionProfile<TrapezoidalMotionProfile>
        implements DynamicFollowable<BasicMoment> {

    public TrapezoidalBasicProfile(RobotSpecs specs, double distance) {
        profile = new TrapezoidalMotionProfile(specs, distance);
    }

    public TrapezoidalBasicProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }

    @Override
    public void update(BasicMoment m) {
        profile.update(m.getTime(), m.getPosition(), m.getVelocity(), m.getAcceleration());
    }
}
