package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.DynamicMotionProfile;

public abstract class DynamicBasicFollowableMotionProfile extends BasicFollowableMotionProfile
        implements DynamicFollowable<BasicMoment> {
    
    @Override
    public void update(BasicMoment m) {
        ((DynamicMotionProfile) profile).update(m.getTime(), m.getPosition(), m.getVelocity(), m.getAcceleration());
    }
}
