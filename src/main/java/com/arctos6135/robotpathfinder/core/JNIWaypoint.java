package com.arctos6135.robotpathfinder.core;

public class JNIWaypoint {

    protected double x;
    protected double y;
    protected double heading;
    protected double velocity = Double.NaN;

    public JNIWaypoint() {}
    public JNIWaypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public JNIWaypoint(double x, double y, double heading, double velocity) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocity = velocity;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }
    public double getVelocity() {
        return velocity;
    }
}
