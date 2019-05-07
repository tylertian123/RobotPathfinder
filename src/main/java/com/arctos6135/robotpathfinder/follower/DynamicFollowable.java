package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

public interface DynamicFollowable extends Followable {
    public void update(Moment m);
}
