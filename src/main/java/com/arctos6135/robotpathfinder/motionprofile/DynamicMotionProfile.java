package com.arctos6135.robotpathfinder.motionprofile;

public interface DynamicMotionProfile extends MotionProfile {
    public void update(double currentTime, double currentPos, double currentVel, double currentAccel);
}
