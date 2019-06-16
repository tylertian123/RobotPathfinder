package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link BasicTrajectory}.
 */
public class BasicTrajectoryTest {

    /**
     * Performs velocity and acceleration limit testing on a
     * {@link BasicTrajectory}.
     * 
     * This test generates a {@link BasicTrajectory} and loops through all its
     * Moments, ensuring that the absolute value of the velocity and the
     * acceleration never exceeds the limit.
     */
    @Test
    public void testVelocityAndAccelerationLimitBasic() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        for (BasicMoment m : trajectory.getMoments()) {
            if (MathUtils.floatGt(Math.abs(m.getVelocity()), maxV)) {
                fail("The BasicTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (MathUtils.floatGt(Math.abs(m.getAcceleration()), maxA)) {
                fail("The BasicTrajectory exceeded the acceleration limit at time " + m.getTime());
            }
        }
        trajectory.close();
    }

    /**
     * Performs tests on the velocity constraints at the beginning and end waypoints
     * of a {@link BasicTrajectory}.
     * 
     * This test generates a {@link BasicTrajectory} that has its velocity specified
     * through a {@link Waypoint} at the beginning and end. It then tests that the
     * trajectory's velocity meets the constraints at the beginning and end.
     */
    @Test
    public void testBeginningAndEndWaypointEx() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);
        double startVel = helper.getDouble("startVel", maxV);
        double endVel = helper.getDouble("endVel", maxV);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading, startVel),
                new Waypoint(endX, endY, endHeading, endVel), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        assertThat(trajectory.get(0).getVelocity(), closeTo(startVel, MathUtils.getFloatCompareThreshold()));
        assertThat(trajectory.get(trajectory.totalTime()).getVelocity(), closeTo(endVel, MathUtils.getFloatCompareThreshold()));
        trajectory.close();
    }

    /**
     * Performs tests on {@link BasicTrajectory#mirrorLeftRight()}.
     * 
     * This test generates a {@link BasicTrajectory} and calls
     * {@code mirrorLeftRight()} on it twice. It then loops through 100 different
     * points in time and verifies that the twice mirrored trajectory is identical
     * to the original trajectory.
     */
    @Test
    public void testBasicTrajectoryMirrorLeftRight() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory original = new BasicTrajectory(robotSpecs, params);
        BasicTrajectory t = original.mirrorLeftRight();
        BasicTrajectory mirrored = t.mirrorLeftRight();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on {@link BasicTrajectory#mirrorFrontBack()}.
     * 
     * This test generates a {@link BasicTrajectory} and calls
     * {@code mirrorFrontBack()} on it twice. It then loops through 100 different
     * points in time and verifies that the twice mirrored trajectory is identical
     * to the original trajectory.
     */
    @Test
    public void testBasicTrajectoryMirrorFrontBack() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory original = new BasicTrajectory(robotSpecs, params);
        BasicTrajectory t = original.mirrorFrontBack();
        BasicTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on {@link BasicTrajectory#retrace()}.
     * 
     * This test generates a {@link BasicTrajectory} and calls {@code retrace()} on
     * it twice. It then loops through 100 different points in time and verifies
     * that the twice retraced trajectory is identical to the original trajectory.
     */
    @Test
    public void testBasicTrajectoryRetrace() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory original = new BasicTrajectory(robotSpecs, params);
        BasicTrajectory t = original.retrace();
        BasicTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs tests on combining the mirroring methods of {@link BasicTrajectory}
     * ({@link BasicTrajectory#mirrorLeftRight()},
     * {@link BasicTrajectory#mirrorFrontBack()},
     * {@link BasicTrajectory#retrace()}).
     * 
     * This test generates a {@link BasicTrajectory}, and then does these calls in
     * order:
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
    public void testBasicTrajectoryMultipleMirroring() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double startX = helper.getDouble("startX", 1000);
        double startY = helper.getDouble("startY", 1000);
        double endX = helper.getDouble("endX", 1000);
        double endY = helper.getDouble("endY", 1000);
        double startHeading = helper.getDouble("startHeading", Math.PI * 2);
        double endHeading = helper.getDouble("endHeading", Math.PI * 2);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory original = new BasicTrajectory(robotSpecs, params);
        BasicTrajectory t0 = original.mirrorLeftRight();
        BasicTrajectory t1 = t0.mirrorFrontBack();
        BasicTrajectory t2 = t1.retrace();
        BasicTrajectory t3 = t2.retrace();
        BasicTrajectory t4 = t3.mirrorFrontBack();
        BasicTrajectory mirrored = t4.mirrorLeftRight();

        t0.free();
        t1.free();
        t2.free();
        t3.free();
        t4.free();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
        }

        original.close();
        mirrored.close();
    }

    /**
     * Performs impossible constraints exception testing on {@link BasicTrajectory}.
     * 
     * This test tries to generate a {@link BasicTrajectory} using impossible
     * constraints by specifying an unreachable velocity using a {@link Waypoint}.
     * It then asserts that the constructor throws an exception from native code.
     */
    @Test(expected = TrajectoryGenerationException.class)
    public void testBasicTrajectoryGenerationException() {
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

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(startX, startY, startHeading),
                new Waypoint(midX, midY, midHeading, midVel), new Waypoint(endX, endY, endHeading), };
        params.alpha = Math.sqrt((startX - endX) * (startX - endX) + (startY - endY) * (startY - endY));
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;

        BasicTrajectory traj = new BasicTrajectory(robotSpecs, params);
        traj.close();
    }
}
