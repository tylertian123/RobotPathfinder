package com.arctos6135.robotpathfinder.tests;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.Follower.DirectionSource;
import com.arctos6135.robotpathfinder.follower.Follower.DistanceSource;
import com.arctos6135.robotpathfinder.follower.Follower.Motor;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;

import org.junit.Test;

public class FollowerTest {

    public static Follower.TimestampSource timer = () -> {
        return System.currentTimeMillis() / 1000.0;
    };
    public static DistanceSource distancePlaceholder = () -> { return 0d; };
    public static DirectionSource directionPlaceholder = () -> { return 0d; };
    public static Motor motorPlaceholder = (s) -> {};
    
    @Test
    public void testTankDriveFollowerBasic() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);

        Follower follower = new TankDriveFollower(traj, motorPlaceholder, motorPlaceholder, 
                distancePlaceholder, distancePlaceholder, timer, directionPlaceholder, 0, 0, 0, 0, 0);
        follower.initialize();
        follower.run();
        follower.stop();
        
        traj.free();
    }
}
