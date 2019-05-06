package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.BasicFollowable;
import com.arctos6135.robotpathfinder.follower.TankDriveFollowable;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalRotationTankDriveProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;

import org.junit.Test;

public class FollowableMotionProfileTest {

    @Test
    public void testTrapezoidalBasicProfileBasic() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        BasicFollowable f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getAcceleration(), closeTo(maxA, 1e-7));
        assertThat(end.getAcceleration(), closeTo(-maxA, 1e-7));
    }

    @Test
    public void testTrapezoidalBasicProfileBasicReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        BasicFollowable f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, 1e-7));
        assertThat(end.getPosition(), closeTo(distance, 1e-7));
        assertThat(begin.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(end.getVelocity(), closeTo(0.0, 1e-7));
        assertThat(begin.getAcceleration(), closeTo(-maxA, 1e-7));
        assertThat(end.getAcceleration(), closeTo(maxA, 1e-7));
    }

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

        BasicFollowable f = new TrapezoidalBasicProfile(specs, distance);

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

        BasicFollowable f = new TrapezoidalBasicProfile(specs, distance);

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

    @Test
    public void testTrapezoidalTankDriveProfileBasic() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TankDriveFollowable f = new TrapezoidalTankDriveProfile(specs, distance);
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
    public void testTrapezoidalTankDriveProfileBasicReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TankDriveFollowable f = new TrapezoidalTankDriveProfile(specs, distance);
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

        TankDriveFollowable f = new TrapezoidalTankDriveProfile(specs, distance);

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

        TankDriveFollowable f = new TrapezoidalTankDriveProfile(specs, distance);

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

    @Test
    public void testTrapezoidalRotationTankDriveProfileBasic() {
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

        TankDriveFollowable f = new TrapezoidalRotationTankDriveProfile(specs, angle);
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

    @Test
    public void testTrapezoidalRotationTankDriveProfileBasicReversed() {
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

        TankDriveFollowable f = new TrapezoidalRotationTankDriveProfile(specs, angle);
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

    @Test
    public void testTrapezoidalRotationTankDriveProfileAdvanced() {
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

        TankDriveFollowable f = new TrapezoidalRotationTankDriveProfile(specs, angle);

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

    @Test
    public void testTrapezoidalRotationTankDriveProfileAdvancedReversed() {
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

        TankDriveFollowable f = new TrapezoidalRotationTankDriveProfile(specs, angle);

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
