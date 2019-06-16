package com.arctos6135.robotpathfinder.motionprofile;

public interface DynamicMotionProfile extends MotionProfile {
    public boolean update(double currentTime, double currentPos, double currentVel, double currentAccel);
}
