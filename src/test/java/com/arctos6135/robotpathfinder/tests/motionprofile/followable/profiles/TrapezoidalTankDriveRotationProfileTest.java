package com.arctos6135.robotpathfinder.tests.motionprofile.followable.profiles;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveRotationProfile;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalTankDriveRotationProfile}.
 */
public class TrapezoidalTankDriveRotationProfileTest {

    /**
     * Performs basic testing on {@link TrapezoidalTankDriveRotationProfile}.
     * 
     * This test creates a {@link TrapezoidalTankDriveRotationProfile} and asserts
     * that the starting position, velocity, relative facing, and end velocity are
     * all 0, the end position is equal to the calculated end position, the starting
     * acceleration is the max acceleration, the end acceleration is negative the
     * max acceleration for each of the sides, and the end relative facing is as
     * specified.
     */
    @Test
    public void testTrapezoidalTankDriveRotationProfile() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double angle = rand.nextDouble() * Math.PI;
        double baseWidth = rand.nextDouble() * 1000;
        double baseRadius = baseWidth / 2;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] angle: " + angle);
        System.out.println("[INFO] baseWidth: " + baseWidth);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftPosition(), closeTo(-baseRadius * angle, 1e-7));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getLeftAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getLeftAcceleration(), closeTo(maxA, 1e-7));

        assertThat(begin.getRightPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getRightPosition(), closeTo(baseRadius * angle, 1e-7));
        assertThat(begin.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getRightAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getRightAcceleration(), closeTo(-maxA, 1e-7));

        assertThat(begin.getFacingRelative(), closeTo(0.0, 1e-7));
        assertThat(end.getFacingRelative(), closeTo(angle, 1e-7));
    }

    /**
     * Performs basic testing on {@link TrapezoidalTankDriveRotationProfile} using a
     * negative angle.
     * 
     * This test is identical to {@link #testTrapezoidalTankDriveRotationProfile()}
     * except that it uses a negative angle to construct the
     * {@link TrapezoidalTankDriveRotationProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalTankDriveRotationProfileReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double angle = -rand.nextDouble() * Math.PI;
        double baseWidth = rand.nextDouble() * 1000;
        double baseRadius = baseWidth / 2;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] angle: " + angle);
        System.out.println("[INFO] baseWidth: " + baseWidth);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftPosition(), closeTo(-baseRadius * angle, 1e-7));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getLeftAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getLeftAcceleration(), closeTo(-maxA, 1e-7));

        assertThat(begin.getRightPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getRightPosition(), closeTo(baseRadius * angle, 1e-7));
        assertThat(begin.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getRightAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getRightAcceleration(), closeTo(maxA, 1e-7));

        assertThat(begin.getFacingRelative(), closeTo(0.0, 1e-7));
        assertThat(end.getFacingRelative(), closeTo(angle, 1e-7));
    }

    /**
     * Performs testing on many points of a
     * {@link TrapezoidalTankDriveRotationProfile}.
     * 
     * This test creates a {@link TrapezoidalTankDriveRotationProfile} and loops
     * through 1000 points in time, asserting that the position is between 0 and the
     * calculated end position, the velocity is between 0 and the specified max
     * velocity, the acceleration is between the negative max acceleration and max
     * acceleration for both sides, and that the relative facing is always less than
     * or equal to the specified angle.
     */
    @Test
    public void testTrapezoidalTankDriveRotationProfileAdvanced() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double angle = rand.nextDouble() * Math.PI;
        double baseWidth = rand.nextDouble() * 1000;
        double baseRadius = baseWidth / 2;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] angle: " + angle);
        System.out.println("[INFO] baseWidth: " + baseWidth);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);

        double leftDist = -baseRadius * angle;
        double rightDist = -leftDist;
        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(), either(greaterThan((leftDist))).or(closeTo((leftDist), 1e-7)));
            assertThat(m.getLeftPosition(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-m.getLeftVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getLeftVelocity(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getLeftAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getRightPosition(), either(lessThan(rightDist)).or(closeTo(rightDist, 1e-7)));
            assertThat(m.getRightPosition(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(m.getRightVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getRightVelocity(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getRightAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getFacingRelative(), either(lessThan(angle)).or(closeTo(angle, 1e-7)));
        }
    }

    /**
     * Performs testing on many points of a
     * {@link TrapezoidalTankDriveRotationProfile} using a negative angle.
     * 
     * This test is identical to
     * {@link #testTrapezoidalTankDriveRotationProfileAdvanced()} except that it
     * uses a negative angle to construct the
     * {@link TrapezoidalTankDriveRotationProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalTankDriveRotationProfileAdvancedReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double angle = -rand.nextDouble() * Math.PI;
        double baseWidth = rand.nextDouble() * 1000;
        double baseRadius = baseWidth / 2;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] angle: " + angle);
        System.out.println("[INFO] baseWidth: " + baseWidth);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);

        double leftDist = -baseRadius * angle;
        double rightDist = -leftDist;
        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(), either(lessThan(leftDist)).or(closeTo(leftDist, 1e-7)));
            assertThat(m.getLeftPosition(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(m.getLeftVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getLeftVelocity(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getLeftAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getRightPosition(), either(greaterThan((rightDist))).or(closeTo((rightDist), 1e-7)));
            assertThat(m.getRightPosition(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-m.getRightVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getRightVelocity(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getRightAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getFacingRelative(), either(greaterThan(angle)).or(closeTo(angle, 1e-7)));
        }
    }
}
