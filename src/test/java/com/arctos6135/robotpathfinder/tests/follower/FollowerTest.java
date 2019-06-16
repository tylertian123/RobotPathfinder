package com.arctos6135.robotpathfinder.tests.follower;

import static org.hamcrest.Matchers.not;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.closeTo;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

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
     * Performs testing on the 'state' logic of {@link Follower} using a
     * {@link TankDriveFollower}.
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);
        double baseWidth = helper.getDouble("baseWidth", 1000);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA, baseWidth);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory traj = new TankDriveTrajectory(robotSpecs, params);

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

    /**
     * Performs basic tests on the motor outputs of {@link TankDriveFollower}.
     * 
     * This test creates a {@link TankDriveFollower} and asserts the following:
     * <ul>
     * <li>With nonzero PDVA constants, time=totalTime and all sensor readings 0,
     * the motor output is non-zero.</li>
     * <li>With kV=1, kA=0, kD=0 and an arbitrary kP, and no difference between
     * position readings and the desired position, the output is equal to the
     * velocity for that time.</li>
     * <li>With kA=1, kV=0, kA=0 and an arbitary kP, and no difference between
     * position readings and the desired position, the output is equal to the
     * acceleration for that time.
     * </ul>
     */
    @Test
    public void testTankDriveFollowerMotorOutput() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1.0);
        double maxA = helper.getDouble("maxA", 1.0);
        double distance = helper.getDouble("distance", 1000);
        double kP = helper.getDouble("kP", MathUtils.getFloatCompareThreshold(), 1000);
        double kD = helper.getDouble("kD", MathUtils.getFloatCompareThreshold(), 1000);
        double kV = helper.getDouble("kV", MathUtils.getFloatCompareThreshold(), 1000);
        double kA = helper.getDouble("kA", MathUtils.getFloatCompareThreshold(), 1000);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrapezoidalTankDriveProfile profile = new TrapezoidalTankDriveProfile(robotSpecs, distance);

        double checkTime = helper.getDouble("checkTime", profile.totalTime());

        FakeTimer timer = new FakeTimer();
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        Follower<TankDriveMoment> follower = new TankDriveFollower(profile, motor, motor, encoder, encoder, timer, kV,
                kA, kP, kD);

        follower.initialize();
        timer.value = profile.totalTime();
        follower.run();
        assertThat(motor.value, not(closeTo(0.0, MathUtils.getFloatCompareThreshold())));

        follower.stop();
        follower.setGains(1, 0, kP, 0);
        timer.value = 0;
        follower.initialize();
        timer.value = checkTime;
        encoder.value = profile.get(checkTime).getLeftPosition();
        follower.run();
        assertThat(motor.value,
                closeTo(profile.get(checkTime).getLeftVelocity(), MathUtils.getFloatCompareThreshold()));

        follower.stop();
        follower.setGains(0, 1, kP, 0);
        timer.value = 0;
        encoder.value = 0;
        follower.initialize();
        timer.value = checkTime;
        encoder.value = profile.get(checkTime).getLeftPosition();
        follower.run();
        assertThat(motor.value,
                closeTo(profile.get(checkTime).getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
    }
}
