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
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;

import org.junit.Test;

public class TrapezoidalTankDriveProfileTest {

    @Test
    public void testTrapezoidalTankDriveProfile() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getLeftAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getLeftAcceleration(), closeTo(-maxA, 1e-7));

        assertThat(begin.getRightPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getRightPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getRightAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getRightAcceleration(), closeTo(-maxA, 1e-7));
    }

    @Test
    public void testTrapezoidalTankDriveProfileReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getLeftVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getLeftAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getLeftAcceleration(), closeTo(maxA, 1e-7));

        assertThat(begin.getRightPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getRightPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getRightVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getRightAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getRightAcceleration(), closeTo(maxA, 1e-7));
    }

    @Test
    public void testTrapezoidalTankDriveProfileAdvanced() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(), either(lessThan(distance)).or(closeTo(distance, 1e-7)));
            assertThat(m.getLeftPosition(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(m.getLeftVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getLeftVelocity(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getLeftAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getRightPosition(), either(lessThan(distance)).or(closeTo(distance, 1e-7)));
            assertThat(m.getRightPosition(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(m.getRightVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getRightVelocity(), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getRightAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }

    @Test
    public void testTrapezoidalTankDriveProfileAdvancedReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(), either(greaterThan((distance))).or(closeTo((distance), 1e-7)));
            assertThat(m.getLeftPosition(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-m.getLeftVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getLeftVelocity(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getLeftAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));

            assertThat(m.getRightPosition(), either(greaterThan((distance))).or(closeTo((distance), 1e-7)));
            assertThat(m.getRightPosition(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-m.getRightVelocity(), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(m.getRightVelocity(), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(m.getRightAcceleration()), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }
}
