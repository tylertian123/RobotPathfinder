package com.arctos6135.robotpathfinder.motionprofile;

public interface DualMotionProfile {
    public double totalTime();

    public boolean isReversed();

    public double leftPosition(double t);

    public double rightPosition(double t);

    public double leftVelocity(double t);

    public double rightVelocity(double t);

    public double leftAcceleration(double t);

    public double rightAcceleration(double t);
}
