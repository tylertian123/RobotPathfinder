package com.arctos6135.robotpathfinder.motionprofile;

public class DualMotionProfile<T extends MotionProfile> {

    protected T leftProfile, rightProfile;

    public DualMotionProfile(T left, T right) {
        leftProfile = left;
        rightProfile = right;
    }

    public double totalTime() {
        return Math.max(leftProfile.totalTime(), rightProfile.totalTime());
    }

    public boolean isReversed() {
        return leftProfile.isReversed() && rightProfile.isReversed();
    }

    public double leftPosition(double t) {
        return leftProfile.position(t);
    }

    public double rightPosition(double t) {
        return rightProfile.position(t);
    }

    public double leftVelocity(double t) {
        return leftProfile.velocity(t);
    }

    public double rightVelocity(double t) {
        return rightProfile.velocity(t);
    }

    public double leftAcceleration(double t) {
        return leftProfile.acceleration(t);
    }

    public double rightAcceleration(double t) {
        return rightProfile.acceleration(t);
    }
}
