package com.arctos6135.robotpathfinder.core;

public class Waypoint {

    protected double x;
    protected double y;
    protected double heading;
    protected double velocity = Double.NaN;

    public Waypoint() {}
    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Waypoint(double x, double y, double heading, double velocity) {
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
