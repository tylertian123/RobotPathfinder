package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;

import org.junit.Test;

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
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(0.0, 100.0, Math.PI / 2), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        for (BasicMoment m : trajectory.getMoments()) {
            if (Math.abs(m.getVelocity()) > 5.0) {
                fail("The BasicTrajectory exceeded the velocity limit at time " + m.getTime());
            }
            if (Math.abs(m.getAcceleration()) > 3.5) {
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
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        assertThat(trajectory.get(0).getVelocity(), is(1.23));
        assertThat(trajectory.get(trajectory.totalTime()).getVelocity(), is(3.45));
        trajectory.close();
    }
    @Test
    public void testBasicTrajectoryMirrorLeftRight() {
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.mirrorLeftRight();
        BasicTrajectory mirrored = t.mirrorLeftRight();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), 1e-7));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), 1e-7));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
        }

        original.close();
        mirrored.close();
    }

    @Test
    public void testBasicTrajectoryMirrorFrontBack() {
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.mirrorFrontBack();
        BasicTrajectory mirrored = t.mirrorFrontBack();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), 1e-7));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), 1e-7));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
        }

        original.close();
        mirrored.close();
    }

    @Test
    public void testBasicTrajectoryRetrace() {
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory original = new BasicTrajectory(specs, params);
        BasicTrajectory t = original.retrace();
        BasicTrajectory mirrored = t.retrace();
        t.close();

        double dt = original.totalTime() / 100;
        for (int i = 0; i < dt; i++) {
            BasicMoment m0 = original.get(dt * i);
            BasicMoment m1 = mirrored.get(dt * i);

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), 1e-7));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), 1e-7));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
        }

        original.close();
        mirrored.close();
    }

    @Test
    public void testBasicTrajectoryMultipleMirroring() {
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0.0, 0.0, Math.PI / 2, 1.23),
                new Waypoint(0.0, 100.0, Math.PI / 2, 3.45), };
        params.alpha = 40.0;
        params.sampleCount = 1000;
        params.pathType = PathType.QUINTIC_HERMITE;
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

            assertThat(m0.getPosition(), closeTo(m1.getPosition(), 1e-7));
            assertThat(m0.getVelocity(), closeTo(m1.getVelocity(), 1e-7));
            assertThat(m0.getAcceleration(), closeTo(m1.getAcceleration(), 1e-7));
            assertThat(m0.getHeading(), closeTo(m1.getHeading(), 1e-7));
        }

        original.close();
        mirrored.close();
    }

    @Test
    public void testBasicTrajectoryGenerationException() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2, 10),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        boolean exception = false;
        try {
            BasicTrajectory traj = new BasicTrajectory(specs, params);
            traj.close();
        } catch (TrajectoryGenerationException e) {
            exception = true;
        }

        assertThat(exception, is(true));
    }
}
