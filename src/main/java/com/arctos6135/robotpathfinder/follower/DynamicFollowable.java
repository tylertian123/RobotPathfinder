package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

public interface DynamicFollowable<T extends Moment> extends Followable<T> {
    public void update(T m);
}
