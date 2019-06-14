package com.arctos6135.robotpathfinder.tests.follower;

import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;

import org.junit.Test;

/**
 * This class contains tests for the followers of RobotPathfinder.
 */
public class FollowerTest {

    /**
     * A fake {@link Follower.TimestampSource} used for testing.
     * 
     * It returns a user-defined value (default 0).
     */
    public static class FakeTimer implements Follower.TimestampSource {

        public double value = 0;

        @Override
        public double getTimestamp() {
            return value;
        }
    }

    /**
     * A fake {@link Follower.DistanceSource} used for testing.
     * 
     * It returns a user-defined value (default 0).
     */
    public static class FakeEncoder implements Follower.DistanceSource {

        public double value = 0;

        @Override
        public double getDistance() {
            return value;
        }
    }

    /**
     * A fake {@link Follower.DirectionSource} used for testing.
     * 
     * It returns a user-defined value (default 0).
     */
    public static class FakeGyro implements Follower.DirectionSource {

        public double value = 0;

        @Override
        public double getDirection() {
            return value;
        }
    }

    /**
     * A fake {@link Follower.Motor} used for testing.
     * 
     * Every time it is set, the value is stored in a public member variable. The
     * default value of this variable is {@code NaN}.
     */
    public static class FakeMotor implements Follower.Motor {

        public double value = Double.NaN;

        @Override
        public void set(double value) {
            this.value = value;
        }
    }

    /**
     * Performs very basic testing on {@link TankDriveFollower}.
     * 
     * This class creates a {@link TankDriveFollower}, and then calls
     * {@code initialize()}, {@code run()} and {@code stop()} on it.
     */
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

        Follower<TankDriveMoment> follower = new TankDriveFollower(traj, new FakeMotor(), new FakeMotor(),
                new FakeEncoder(), new FakeEncoder(), new FakeTimer(), new FakeGyro(), 0, 0, 0, 0, 0);
        follower.initialize();
        follower.run();
        follower.stop();

        traj.free();
    }

    /**
     * Performs testing on the 'state' logic of {@link TankDriveFollower}.
     * 
     * This test creates a {@link TankDriveFollower} and asserts the following:
     * <ul>
     * <li>The follower is not finished or running upon creation.</li>
     * <li>The follower is not finished and is running after being initialized.</li>
     * <li>The follower is still running and not finished after a single iteration
     * of {@code run()}.</li>
     * <li>The follower is finished and not running when the time reaches the end of
     * the trajectory.</li>
     * <li>The follower does not reinitialize itself after being stopped.</li>
     * <li>The follower is finished and not running after being stopped.</li>
     * </ul>
     */
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
        Follower<TankDriveMoment> follower = new TankDriveFollower(traj, new FakeMotor(), new FakeMotor(),
                new FakeEncoder(), new FakeEncoder(), timer, new FakeGyro(), 0, 0, 0, 0, 0);
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
