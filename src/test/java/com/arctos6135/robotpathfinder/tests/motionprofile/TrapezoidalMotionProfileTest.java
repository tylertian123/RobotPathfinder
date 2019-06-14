package com.arctos6135.robotpathfinder.tests.motionprofile;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalMotionProfile}.
 */
public class TrapezoidalMotionProfileTest {

    /**
     * Performs basic testing on {@link TrapezoidalMotionProfile}.
     * 
     * This test constructs a {@link TrapezoidalMotionProfile}, and asserts that the
     * starting position and velocity and end velocity are all 0, the end position
     * is as expected, the starting acceleration is the max acceleration, and the
     * end acceleration is negative the max acceleration.
     */
    @Test
    public void testTrapezoidalMotionProfile() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat(profile.position(profile.totalTime()), closeTo(distance, 1e-7));
        assertThat(profile.position(0), closeTo(0.0, 1e-7));
        assertThat(profile.velocity(0), closeTo(0.0, 1e-7));
        assertThat(profile.velocity(profile.totalTime()), closeTo(0.0, 1e-7));
        assertThat(profile.acceleration(0), closeTo(maxA, 1e-7));
        assertThat(profile.acceleration(profile.totalTime()), closeTo(-maxA, 1e-7));
    }

    /**
     * Performs basic testing on {@link TrapezoidalMotionProfile} using a negative
     * position.
     * 
     * This test is identical to {@link #testTrapezoidalMotionProfile()} except that
     * it uses a negative position to construct the {@link TrapezoidalMotionProfile}
     * so that it goes backwards.
     */
    @Test
    public void testTrapezoidalMotionProfileReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat(profile.position(profile.totalTime()), closeTo(distance, 1e-7));
        assertThat(profile.position(0), closeTo(0.0, 1e-7));
        assertThat(profile.velocity(0), closeTo(0.0, 1e-7));
        assertThat(profile.velocity(profile.totalTime()), closeTo(0.0, 1e-7));
        assertThat(profile.acceleration(0), closeTo(-maxA, 1e-7));
        assertThat(profile.acceleration(profile.totalTime()), closeTo(maxA, 1e-7));
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalMotionProfile}.
     * 
     * This test creates a {@link TrapezoidalMotionProfile} and loops through 1000
     * points in time, asserting that the position is between 0 and the specified
     * end position, the velocity is between 0 and the specified max velocity, and
     * the acceleration is between the negative max acceleration and max
     * acceleration.
     */
    @Test
    public void testTrapezoidalMotionProfileAdvanced() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for (double t = 0; t < profile.totalTime(); t += dt) {
            assertThat(profile.position(t), either(lessThan(distance)).or(closeTo(distance, 1e-7)));
            assertThat(profile.position(t), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(profile.velocity(t), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(profile.velocity(t), either(greaterThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(profile.acceleration(t)), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalMotionProfile} using a
     * negative position.
     * 
     * This test is identical to {@link #testTrapezoidalMotionProfileAdvanced()}
     * except that it uses a negative position to construct the
     * {@link TrapezoidalBasicProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalMotionProfileAdvancedReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for (double t = 0; t < profile.totalTime(); t += dt) {
            assertThat(profile.position(t), either(greaterThan((distance))).or(closeTo((distance), 1e-7)));
            assertThat(profile.position(t), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-profile.velocity(t), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(profile.velocity(t), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(profile.acceleration(t)), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }

    /**
     * Performs basic testing on
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)}.
     * 
     * This test constructs a {@link TrapezoidalMotionProfile} and calls its
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)
     * update()} method with a set of randomly generated values. It asserts that
     * after updating, the end velocity is 0; if the update() method returned true
     * for overshoot, it will check if the end position is greater than the original
     * specified position. Otherwise, it will check if the end position is equal to
     * the original specified position.
     */
    @Test
    public void testTrapezoidalMotionProfileUpdate() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = rand.nextDouble() * profile.totalTime();
        double updatePos = rand.nextDouble() * distance;
        double updateVel = rand.nextDouble() * maxV;

        System.out.println("[INFO] updateTime: " + updateTime);
        System.out.println("[INFO] updatePos: " + updatePos);
        System.out.println("[INFO] updateVel: " + updateVel);

        // TrapezoidalMotionProfiles completely ignore the acceleration when updating
        // since theres no constraint on how fast it changes
        // Just use 0 here as a placeholder
        boolean overshoot = profile.update(updateTime, updatePos, updateVel, 0);

        double totalTime = profile.totalTime();
        assertThat(profile.velocity(totalTime), closeTo(0, 1e-7));
        if (overshoot) {
            assertThat(profile.position(totalTime), greaterThan(distance));
        } else {
            assertThat(profile.position(totalTime), closeTo(distance, 1e-7));
        }
    }

    /**
     * Performs basic testing on
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)}.
     * 
     * This test is identical to {@link #testTrapezoidalMotionProfileUpdate()},
     * except that it uses a negative position so that the generated profile is
     * reversed.
     */
    @Test
    public void testTrapezoidalMotionProfileUpdateReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = rand.nextDouble() * profile.totalTime();
        double updatePos = rand.nextDouble() * distance;
        double updateVel = rand.nextDouble() * -maxV;

        System.out.println("[INFO] updateTime: " + updateTime);
        System.out.println("[INFO] updatePos: " + updatePos);
        System.out.println("[INFO] updateVel: " + updateVel);

        // TrapezoidalMotionProfiles completely ignore the acceleration when updating
        // since theres no constraint on how fast it changes
        // Just use 0 here as a placeholder
        boolean overshoot = profile.update(updateTime, updatePos, updateVel, 0);

        double totalTime = profile.totalTime();
        assertThat(profile.velocity(totalTime), closeTo(0, 1e-7));
        if (overshoot) {
            // Assert that it is more negative than the distance
            assertThat(profile.position(totalTime), lessThan(distance));
        } else {
            assertThat(profile.position(totalTime), closeTo(distance, 1e-7));
        }
    }

    /**
     * Performs basic testing on the overshoot handling of
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)}.
     * 
     * This test constructs a {@link TrapezoidalMotionProfile}, and calls
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)
     * update()} with the current position being the end position specified by the
     * constructor. It then asserts that the update causes the profile to overshoot,
     * the end velocity is still 0, and the final position is correct based on
     * calculations.
     */
    @Test
    public void testTrapezoidalMotionProfileUpdateOvershoot() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = rand.nextDouble() * profile.totalTime();
        double updateVel = rand.nextDouble() * maxV;

        System.out.println("[INFO] updateTime: " + updateTime);
        System.out.println("[INFO] updateVel: " + updateVel);

        boolean overshoot = profile.update(updateTime, distance, updateVel, 0);
        double totalTime = profile.totalTime();

        assertThat(overshoot, is(true));
        assertThat(profile.velocity(totalTime), closeTo(0, 1e-7));

        // Calculate the overshoot distance using kinematic forumlas
        double overshootDist = -(updateVel * updateVel) / (2 * -maxA);
        assertThat(profile.position(totalTime), closeTo(distance + overshootDist, 1e-7));
    }

    /**
     * Performs basic testing on the overshoot handling of
     * {@link TrapezoidalMotionProfile#update(double, double, double, double)}.
     * 
     * This test is identical to
     * {@link #testTrapezoidalMotionProfileUpdateOvershoot()}, except that it uses a
     * negative position so that the generated profile is reversed.
     */
    @Test
    public void testTrapezoidalMotionProfileUpdateOvershootReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = rand.nextDouble() * profile.totalTime();
        double updateVel = rand.nextDouble() * -maxV;

        System.out.println("[INFO] updateTime: " + updateTime);
        System.out.println("[INFO] updateVel: " + updateVel);

        boolean overshoot = profile.update(updateTime, distance, updateVel, 0);
        double totalTime = profile.totalTime();

        assertThat(overshoot, is(true));
        assertThat(profile.velocity(totalTime), closeTo(0, 1e-7));

        // Calculate the overshoot distance using kinematic forumlas
        double overshootDist = -(updateVel * updateVel) / (2 * maxA);
        assertThat(profile.position(totalTime), closeTo(distance + overshootDist, 1e-7));
    }
}
