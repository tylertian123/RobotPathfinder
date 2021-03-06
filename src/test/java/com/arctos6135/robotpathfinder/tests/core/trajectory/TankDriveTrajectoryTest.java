package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

/**
 * This class contains tests for {@link TankDriveTrajectory}.
 * 
 * @author Tyler Tian
 */
public class TankDriveTrajectoryTest {

    @Rule
    public TestName testName = new TestName();

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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        TankDriveTrajectory trajectory = new TankDriveTrajectory(specs, params);

        for (TankDriveMoment m : trajectory.getMoments()) {
            if (MathUtils.floatGt(Math.abs(m.getLeftVelocity()), specs.getMaxVelocity())) {
                fail("The left wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (MathUtils.floatGt(Math.abs(m.getRightVelocity()), specs.getMaxVelocity())) {
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        // Generate and mirror
        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = original.mirrorLeftRight();
        TankDriveTrajectory mirrored = t.mirrorLeftRight();
        // Make sure to close to avoid a memory leak
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat("Left position should be the same in both trajectories", m0.getLeftPosition(),
                    closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right position should be the same in both trajectories", m0.getRightPosition(),
                    closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left velocity should be the same in both trajectories", m0.getLeftVelocity(),
                    closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right velocity should be the same in both trajectories", m0.getRightVelocity(),
                    closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left acceleration should be the same in both trajectories", m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right acceleration should be the same in both trajectories", m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = original.mirrorFrontBack();
        TankDriveTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat("Left position should be the same in both trajectories", m0.getLeftPosition(),
                    closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right position should be the same in both trajectories", m0.getRightPosition(),
                    closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left velocity should be the same in both trajectories", m0.getLeftVelocity(),
                    closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right velocity should be the same in both trajectories", m0.getRightVelocity(),
                    closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left acceleration should be the same in both trajectories", m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right acceleration should be the same in both trajectories", m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = original.retrace();
        TankDriveTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat("Left position should be the same in both trajectories", m0.getLeftPosition(),
                    closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right position should be the same in both trajectories", m0.getRightPosition(),
                    closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left velocity should be the same in both trajectories", m0.getLeftVelocity(),
                    closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right velocity should be the same in both trajectories", m0.getRightVelocity(),
                    closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left acceleration should be the same in both trajectories", m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right acceleration should be the same in both trajectories", m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper);

        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
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

            assertThat("Left position should be the same in both trajectories", m0.getLeftPosition(),
                    closeTo(m1.getLeftPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right position should be the same in both trajectories", m0.getRightPosition(),
                    closeTo(m1.getRightPosition(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left velocity should be the same in both trajectories", m0.getLeftVelocity(),
                    closeTo(m1.getLeftVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right velocity should be the same in both trajectories", m0.getRightVelocity(),
                    closeTo(m1.getRightVelocity(), MathUtils.getFloatCompareThreshold()));
            assertThat("Left acceleration should be the same in both trajectories", m0.getLeftAcceleration(),
                    closeTo(m1.getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Right acceleration should be the same in both trajectories", m0.getRightAcceleration(),
                    closeTo(m1.getRightAcceleration(), MathUtils.getFloatCompareThreshold()));
            assertThat("Heading should be the same in both trajectories", m0.getHeading(),
                    closeTo(m1.getHeading(), MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper,
                TrajectoryTestingUtils.getRandomWaypoints(helper, 3));

        double midVel = helper.getDouble("midVel", specs.getMaxVelocity() * 1.1, specs.getMaxVelocity() * 5);
        Waypoint mid = params.waypoints[1];
        params.waypoints[1] = new Waypoint(mid.getX(), mid.getY(), mid.getHeading(), midVel);

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
        traj.close();
    }

    /**
     * Performs basic testing on {@link TankDriveTrajectory#getPosition(double)}.
     * 
     * This test calls the method for both the starting and ending times to make
     * sure it is equal to the position of the waypoints.
     */
    @Test
    public void testTankDriveTrajectoryGetPosition() {
        TestHelper helper = new TestHelper(getClass(), testName);

        RobotSpecs specs = TrajectoryTestingUtils.getRandomRobotSpecs(helper, true);
        Waypoint[] waypoints = TrajectoryTestingUtils.getRandomWaypoints(helper, 2);
        TrajectoryParams params = TrajectoryTestingUtils.getRandomTrajectoryParams(helper, waypoints);

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);

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
