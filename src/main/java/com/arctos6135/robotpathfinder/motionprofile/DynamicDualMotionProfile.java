package com.arctos6135.robotpathfinder.motionprofile;

public class DynamicDualMotionProfile<T extends DynamicMotionProfile> extends DualMotionProfile<T> {

    public DynamicDualMotionProfile(T left, T right) {
        super(left, right);
    }

    public void updateLeft(double currentTime, double currentPos, double currentVel, double currentAccel) {
        leftProfile.update(currentTime, currentPos, currentVel, currentAccel);
    }
    public void updateRight(double currentTime, double currentPos, double currentVel, double currentAccel) {
        rightProfile.update(currentTime, currentPos, currentVel, currentAccel);
    }
}
