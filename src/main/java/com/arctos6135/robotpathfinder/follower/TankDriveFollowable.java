package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;

public interface TankDriveFollowable extends Followable {
    @Override
    public TankDriveMoment get(double t);
}
