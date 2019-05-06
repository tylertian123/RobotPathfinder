package com.arctos6135.robotpathfinder.motionprofile;

public interface DynamicMotionProfile extends MotionProfile {
    public void update(double currentTime, double currentDist, double currentVel, double currentAccel);
}
