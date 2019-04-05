package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

public interface Followable {
    public Moment get(double t);
    public double totalTime();
}
