package com.arctos6135.robotpathfinder.motionprofile;

public interface DynamicDualMotionProfile extends DualMotionProfile {
    public void updateLeft(double currentTime, double currentPos, double currentVel, double currentAccel);
    public void updateRight(double currentTime, double currentPos, double currentVel, double currentAccel);
}
