package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.core.Is.is;
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

import org.junit.Test;

/**
 * This class contains tests for {@link TankDriveTrajectory}.
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
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(0.0, 100.0, Math.PI / 2), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);

        for (TankDriveMoment m : trajectory.getMoments()) {
            if (Math.abs(m.getLeftVelocity()) > 5.0) {
                fail("The left wheel of the TankDriveTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (Math.abs(m.getRightVelocity()) > 5.0) {
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
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
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

            assertThat(m0.getLeftPosition(), is(m1.getLeftPosition()));
            assertThat(m0.getRightPosition(), is(m1.getRightPosition()));
            assertThat(m0.getLeftVelocity(), is(m1.getLeftVelocity()));
            assertThat(m0.getRightVelocity(), is(m1.getRightVelocity()));
            assertThat(m0.getLeftAcceleration(), is(m1.getLeftAcceleration()));
            assertThat(m0.getRightAcceleration(), is(m1.getRightAcceleration()));
            assertThat(m0.getHeading(), is(m1.getHeading()));
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
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = original.mirrorFrontBack();
        TankDriveTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), 1e-7));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), 1e-7));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), 1e-7));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), 1e-7));
            assertThat(m0.getLeftAcceleration(), closeTo(m1.getLeftAcceleration(), 1e-7));
            assertThat(m0.getRightAcceleration(), closeTo(m1.getRightAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
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
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory original = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = original.retrace();
        TankDriveTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            TankDriveMoment m0 = original.get(dt * i);
            TankDriveMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), 1e-7));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), 1e-7));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), 1e-7));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), 1e-7));
            assertThat(m0.getLeftAcceleration(), closeTo(m1.getLeftAcceleration(), 1e-7));
            assertThat(m0.getRightAcceleration(), closeTo(m1.getRightAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
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
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
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

            assertThat(m0.getLeftPosition(), closeTo(m1.getLeftPosition(), 1e-7));
            assertThat(m0.getRightPosition(), closeTo(m1.getRightPosition(), 1e-7));
            assertThat(m0.getLeftVelocity(), closeTo(m1.getLeftVelocity(), 1e-7));
            assertThat(m0.getRightVelocity(), closeTo(m1.getRightVelocity(), 1e-7));
            assertThat(m0.getLeftAcceleration(), closeTo(m1.getLeftAcceleration(), 1e-7));
            assertThat(m0.getRightAcceleration(), closeTo(m1.getRightAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
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
    @Test
    public void testTankDriveTrajectoryGenerationException() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2, 10),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        boolean exception = false;
        try {
            TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
            traj.close();
        } catch (TrajectoryGenerationException e) {
            exception = true;
        }

        assertThat(exception, is(true));
    }
}
