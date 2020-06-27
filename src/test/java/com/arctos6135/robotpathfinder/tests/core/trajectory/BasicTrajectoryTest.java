package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

/**
 * This class contains tests for {@link BasicTrajectory}.
 * 
 * @author Tyler Tian
 */
public class BasicTrajectoryTest {

    @Rule
    public TestName testName = new TestName();

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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);
        BasicTrajectory trajectory = new BasicTrajectory(specs, params);

        for (BasicMoment m : trajectory.getMoments()) {
            if (MathUtils.floatGt(Math.abs(m.getVelocity()), specs.getMaxVelocity())) {
                fail("The BasicTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (MathUtils.floatGt(Math.abs(m.getAcceleration()), specs.getMaxAcceleration())) {
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        double startVel = helper.getDouble("startVel", specs.getMaxVelocity());
        double endVel = helper.getDouble("endVel", specs.getMaxVelocity());

        Waypoint start = params.waypoints[0];
        Waypoint end = params.waypoints[params.waypoints.length - 1];
        params.waypoints[0] = new Waypoint(start.getX(), start.getY(), start.getHeading(), startVel);
        params.waypoints[params.waypoints.length - 1] = new Waypoint(end.getX(), end.getY(), end.getHeading(), endVel);

        BasicTrajectory trajectory;
        try {
            trajectory = new BasicTrajectory(specs, params);
        } catch (TrajectoryGenerationException e) {
            // Oops! Looks like our randomly generated values were too harsh with their
            // requirements and the trajectory is impossible.
            // Just exit here but leave a message
            helper.logMessage("Warning: TrajectoryGenerationException was thrown! Exiting test...");
            return;
        }

        assertThat("Starting velocity should match", trajectory.get(0).getVelocity(),
                closeTo(startVel, MathUtils.getFloatCompareThreshold()));
        assertThat("Ending velocity should match", trajectory.get(trajectory.totalTime()).getVelocity(),
                closeTo(endVel, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.mirrorLeftRight();
        BasicTrajectory mirrored = t.mirrorLeftRight();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat("Position should be the same in both trajectories", m0.getPosition(),
                    closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Velocity should be the same in both trajectories", m0.getVelocity(),
                    closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Acceleration should be the same in both trajectories", m0.getAcceleration(),
                    closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.mirrorFrontBack();
        BasicTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat("Position should be the same in both trajectories", m0.getPosition(),
                    closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Velocity should be the same in both trajectories", m0.getVelocity(),
                    closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Acceleration should be the same in both trajectories", m0.getAcceleration(),
                    closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.retrace();
        BasicTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat("Position should be the same in both trajectories", m0.getPosition(),
                    closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Velocity should be the same in both trajectories", m0.getVelocity(),
                    closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Acceleration should be the same in both trajectories", m0.getAcceleration(),
                    closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        BasicTrajectory original = new BasicTrajectory(specs, params);
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

            assertThat("Position should be the same in both trajectories", m0.getPosition(),
                    closeTo(m1.getPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Velocity should be the same in both trajectories", m0.getVelocity(),
                    closeTo(m1.getVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Acceleration should be the same in both trajectories", m0.getAcceleration(),
                    closeTo(m1.getAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper,
                TrajectoryTestingUtils.getRandomWaypoints(helper, 3));

        double midVel = helper.getDouble("midVel", specs.getMaxVelocity() * 1.1, specs.getMaxVelocity() * 5);
        Waypoint mid = params.waypoints[1];
        params.waypoints[1] = new Waypoint(mid.getX(), mid.getY(), mid.getHeading(), midVel);

        BasicTrajectory traj = new BasicTrajectory(specs, params);
        traj.close();
    }

    /**
     * Performs basic testing on {@link BasicTrajectory#getPosition(double)}.
     * 
     * This test calls the method for both the starting and ending times to make
     * sure it is equal to the position of the waypoints.
     */
    @Test
    public void testBasicTrajectoryGetPosition() {
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, false);
        Waypoint[] waypoints = TrajectoryTestingUtils.getRandomWaypoints(helper, 2);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper, waypoints);

        BasicTrajectory traj = new BasicTrajectory(specs, params);

        Waypoint pos = traj.getPosition(0);
        assertThat("The x should be the same for the first waypoint", pos.getX(),
                closeTo(waypoints[0].getX(), MathUtils.getFloatCompareThreshold()));
        assertThat("The y should be the same for the first waypoint", pos.getY(),
                closeTo(waypoints[0].getY(), MathUtils.getFloatCompareThreshold()));
        assertThat("The heading should be the same for the first waypoint", pos.getHeading(),
                closeTo(waypoints[0].getHeading(), MathUtils.getFloatCompareThreshold()));

        pos = traj.getPosition(traj.totalTime());
        assertThat("The x should be the same for the second waypoint", pos.getX(),
                closeTo(waypoints[1].getX(), MathUtils.getFloatCompareThreshold()));
        assertThat("The y should be the same for the second waypoint", pos.getY(),
                closeTo(waypoints[1].getY(), MathUtils.getFloatCompareThreshold()));
        assertThat("The heading should be the same for the second waypoint", pos.getHeading(),
                closeTo(waypoints[1].getHeading(), MathUtils.getFloatCompareThreshold()));

        traj.close();
    }
}
