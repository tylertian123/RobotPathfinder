package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link TankDriveTrajectory}.
 * 
 * @author Tyler Tian
 */
public class TankDriveTrajectoryTest {

    /**
     * Performs velocity limit testing on a {@link TankDriveTrajectory}.
     * 
     * This test generates a {@link TankDriveTrajectory} and loops through all its
     * Moments, ensuring that the absolute value of the velocity never exceeds the
     * limit.
     * 
     * Note that unlike the test for {@link BasicTrajectory}, this does not test for
     * acceleration, as it cannot be limited at turns by design.
     */
    @Test
    public void testVelocityLimitTank() {
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
        TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);

        for (TankDriveMoment m : trajectory.getMoments()) {
            if (MathUtils.floatGt(Math.abs(m.getLeftVelocity()), maxV)) {
                fail("The left wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (MathUtils.floatGt(Math.abs(m.getRightVelocity()), maxV)) {
                fail("The right wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
        }
        trajectory.close();
    }

    /**
     * Performs tests on {@link TankDriveTrajectory#mirrorLeftRight()}.
     * 
     * This test generates a {@link TankDriveTrajectory} and calls
     * {@code mirrorLeftRight()} on it twice. It then loops through 100 different
     * points in time and verifies that the twice mirrored trajectory is identical
     * to the original trajectory.
     */
    @Test
    public void testTankDriveTrajectoryMirrorLeftRight() {
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
        // Generate and mirror
        TankDriveTrajectory original = new TankDriveTrajectory(robotSpecs, params);
        TankDriveTrajectory t = original.mirrorLeftRight();
        TankDriveTrajectory mirrored = t.mirrorLeftRight();
        // Make sure to close to avoid a memory leak
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftAcceleration(), closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightAcceleration(), closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on {@link TankDriveTrajectory#mirrorFrontBack()}.
     * 
     * This test generates a {@link TankDriveTrajectory} and calls
     * {@code mirrorFrontBack()} on it twice. It then loops through 100 different
     * points in time and verifies that the twice mirrored trajectory is identical
     * to the original trajectory.
     */
    @Test
    public void testTankDriveTrajectoryMirrorFrontBack() {
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

        TankDriveTrajectory original = new TankDriveTrajectory(robotSpecs, params);
        TankDriveTrajectory t = original.mirrorFrontBack();
        TankDriveTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on {@link TankDriveTrajectory#retrace()}.
     * 
     * This test generates a {@link TankDriveTrajectory} and calls {@code retrace()}
     * on it twice. It then loops through 100 different points in time and verifies
     * that the twice retraced trajectory is identical to the original trajectory.
     */
    @Test
    public void testTankDriveTrajectoryRetrace() {
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

        TankDriveTrajectory original = new TankDriveTrajectory(robotSpecs, params);
        TankDriveTrajectory t = original.retrace();
        TankDriveTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on combining the mirroring methods of
     * {@link TankDriveTrajectory} ({@link TankDriveTrajectory#mirrorLeftRight()},
     * {@link TankDriveTrajectory#mirrorFrontBack()},
     * {@link TankDriveTrajectory#retrace()}).
     * 
     * This test generates a {@link TankDriveTrajectory}, and then does these calls
     * in order:
     * <ol>
     * <li>{@code mirrorLeftRight()}</li>
     * <li>{@code mirrorFrontBack()}</li>
     * <li>{@code retrace()}</li>
     * <li>{@code retrace()}</li>
     * <li>{@code mirrorFrontBack()}</li>
     * <li>{@code mirrorLeftRight()}</li>
     * </ol>
     * It then loops through 100 different points in time making sure that the
     * result is identical to the original trajectory.
     */
    @Test
    public void testTankDriveTrajectoryMultipleMirroring() {
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

        TankDriveTrajectory original = new TankDriveTrajectory(robotSpecs, params);
        TankDriveTrajectory t0 = original.mirrorLeftRight();
        TankDriveTrajectory t1 = t0.mirrorFrontBack();
        TankDriveTrajectory t2 = t1.retrace();
        TankDriveTrajectory t3 = t2.retrace();
        TankDriveTrajectory t4 = t3.mirrorFrontBack();
        TankDriveTrajectory mirrored = t4.mirrorLeftRight();

        t0.free();
        t1.free();
        t2.free();
        t3.free();
        t4.free();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs impossible constraints exception testing on
     * {@link TankDriveTrajectory}.
     * 
     * This test tries to generate a {@link TankDriveTrajectory} using impossible
     * constraints by specifying an unreachable velocity using a {@link Waypoint}.
     * It then asserts that the constructor throws an exception from native code.
     */
    @Test(expected = TrajectoryGenerationException.class)
    public void testTankDriveTrajectoryGenerationException() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);
        double midX = helper.getDouble("midX", 1000);
        double midY = helper.getDouble("midY", 1000);
        double midHeading = helper.getDouble("midHeading", Math.PI * 2);
        double midVel = helper.getDouble("midVel", maxV * 1.1, maxV * 5);
        double baseWidth = helper.getDouble("baseWidth", 1000);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA, baseWidth);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(midX, midY, midHeading, midVel), new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        TankDriveTrajectory traj = new TankDriveTrajectory(robotSpecs, params);
        traj.close();
    }
}
