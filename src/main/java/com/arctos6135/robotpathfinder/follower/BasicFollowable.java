package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;

public interface BasicFollowable extends Followable {
    @Override
    public BasicMoment get(double t);
}