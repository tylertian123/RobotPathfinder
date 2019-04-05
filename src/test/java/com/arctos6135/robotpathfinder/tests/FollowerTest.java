package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

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

    public static class FakeTimer implements Follower.TimestampSource {

        public double value = 0;

        @Override
        public double getTimestamp() {
            return value;
        }
    }

    public static DistanceSource distancePlaceholder = () -> {
        return 0d;
    };
    public static DirectionSource directionPlaceholder = () -> {
        return 0d;
    };
    public static Motor motorPlaceholder = (s) -> {
    };

    @Test
    public void testTankDriveFollowerBasic() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);

        Follower follower = new TankDriveFollower(traj, motorPlaceholder, motorPlaceholder, distancePlaceholder,
                distancePlaceholder, new FakeTimer(), directionPlaceholder, 0, 0, 0, 0, 0);
        follower.initialize();
        follower.run();
        follower.stop();

        traj.free();
    }

    @Test
    public void testTankDriveFollowerState() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);

        FakeTimer timer = new FakeTimer();
        Follower follower = new TankDriveFollower(traj, motorPlaceholder, motorPlaceholder, distancePlaceholder,
                distancePlaceholder, timer, directionPlaceholder, 0, 0, 0, 0, 0);
        // Assertion: Follower is not finished or running initially
        assertThat(follower.isFinished(), is(false));
        assertThat(follower.isRunning(), is(false));
        // Assertion: Follower is not finished and is running after initialization
        follower.initialize();
        assertThat(follower.isFinished(), is(false));
        assertThat(follower.isRunning(), is(true));
        // Assertion: Follower is not finished and is running after a single iteration
        follower.run();
        assertThat(follower.isFinished(), is(false));
        assertThat(follower.isRunning(), is(true));
        // Assertion: Follower is finished and is not running after the trajectory
        // finishes
        timer.value = traj.totalTime() + 1;
        follower.run();
        assertThat(follower.isFinished(), is(true));
        assertThat(follower.isRunning(), is(false));
        // Assertion: Follower does not reinitialize after being stopped
        follower.run();
        assertThat(follower.isFinished(), is(true));
        assertThat(follower.isRunning(), is(false));
        // Assertion: Follower is finished and is not running after being stopped
        follower.stop();
        assertThat(follower.isFinished(), is(true));
        assertThat(follower.isRunning(), is(false));

        traj.free();
    }
}
