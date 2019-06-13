package com.arctos6135.robotpathfinder.tests.motionprofile.followable.profiles;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalBasicProfile}.
 */
public class TrapezoidalBasicProfileTest {

    /**
     * Performs basic testing on {@link TrapezoidalBasicProfile}.
     * 
     * This test creates a {@link TrapezoidalBasicProfile} and asserts that the
     * starting position and velocity and end velocity are all 0, the end position
     * is as expected, the starting acceleration is the max acceleration, and the
     * end acceleration is negative the max acceleration.
     */
    @Test
    public void testTrapezoidalBasicProfile() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getAcceleration(), closeTo(-maxA, 1e-7));
    }

    /**
     * Performs basic testing on {@link TrapezoidalBasicProfile} using a negative
     * position.
     * 
     * This test is identical to {@link #testTrapezoidalBasicProfile()} except that
     * it uses a negative position to construct the {@link TrapezoidalBasicProfile}
     * so that it goes backwards.
     */
    @Test
    public void testTrapezoidalBasicProfileReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getAcceleration(), closeTo(maxA, 1e-7));
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalBasicProfile}.
     * 
     * This test creates a {@link TrapezoidalBasicProfile} and loops through 1000
     * points in time, asserting that the position is between 0 and the specified
     * end position, the velocity is between 0 and the specified max velocity, and
     * the acceleration is between the negative max acceleration and max
     * acceleration.
     */
    @Test
    public void testTrapezoidalBasicProfileAdvanced() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            BasicMoment m = f.get(t);
            assertThat(m.getPosition(), either(lessThan(distance)).or(closeTo(distance, 1e-7)));
            assertThat(m.getPosition(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(m.getVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getVelocity(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalBasicProfile} using a
     * negative position.
     * 
     * This test is identical to {@link #testTrapezoidalBasicProfileAdvanced()}
     * except that it uses a negative position to construct the
     * {@link TrapezoidalBasicProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalBasicProfileAdvancedReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            BasicMoment m = f.get(t);
            assertThat(m.getPosition(), either(greaterThan((distance))).or(closeTo((distance), 1e-7)));
            assertThat(m.getPosition(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-m.getVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getVelocity(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }
}
