package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

public interface Followable<T extends Moment> {
    public T get(double t);
    public double totalTime();
}
