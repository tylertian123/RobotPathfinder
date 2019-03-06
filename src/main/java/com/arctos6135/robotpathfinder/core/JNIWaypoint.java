package com.arctos6135.robotpathfinder.core;

public class JNIWaypoint {

    public double x;
    public double y;
    public double heading;
    public double velocity = Double.NaN;

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
}
