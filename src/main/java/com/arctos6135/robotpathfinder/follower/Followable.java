package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

@FunctionalInterface
public interface Followable {
    public Moment get(double t);
}
