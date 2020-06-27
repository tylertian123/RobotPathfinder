package com.arctos6135.robotpathfinder.tests.motionprofile;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.DynamicMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

/**
 * This class contains tests for {@link TrapezoidalMotionProfile}.
 * 
 * @author Tyler Tian
 */
public class TrapezoidalMotionProfileTest {

    @Rule
    public TestName testName = new TestName();

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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat("Position at end time should be close to the specified position",
                profile.position(profile.totalTime()), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat("Position at the start time should be 0", profile.position(0),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Velocity at the start time should be 0", profile.velocity(0),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Velocity at the end time should be 0", profile.velocity(profile.totalTime()),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Acceleration at the start time should be max acceleration", profile.acceleration(0),
                closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat("Acceleration at the end time should be negative max acceleration",
                profile.acceleration(profile.totalTime()), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat("Position at end time should be close to the specified position",
                profile.position(profile.totalTime()), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat("Position at the start time should be 0", profile.position(0),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Velocity at the start time should be 0", profile.velocity(0),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Velocity at the end time should be 0", profile.velocity(profile.totalTime()),
                closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat("Acceleration at the start time should be negative max acceleration", profile.acceleration(0),
                closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat("Acceleration at the end time should be max acceleration", profile.acceleration(profile.totalTime()),
                closeTo(maxA, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for (double t = 0; t < profile.totalTime(); t += dt) {
            assertThat("position should be within the expected range", profile.position(t),
                    either(lessThan(distance)).or(closeTo(distance, MathUtils.getFloatCompareThreshold())));
            assertThat("position should be within the expected range", profile.position(t),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat("velocity should be within the expected range", profile.velocity(t),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat("velocity should be within the expected range", profile.velocity(t),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat("acceleration should be within the expected range", Math.abs(profile.acceleration(t)),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for (double t = 0; t < profile.totalTime(); t += dt) {
            assertThat("position should be within the expected range", profile.position(t),
                    either(greaterThan((distance))).or(closeTo((distance), MathUtils.getFloatCompareThreshold())));
            assertThat("position should be within the expected range", profile.position(t),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat("velocity should be within the expected range", -profile.velocity(t),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat("velocity should be within the expected range", profile.velocity(t),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat("acceleration should be within the expected range", Math.abs(profile.acceleration(t)),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);
        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = helper.getDouble("updateTime", profile.totalTime());
        double updatePos = helper.getDouble("updatePos", distance);
        double updateVel = helper.getDouble("updateVel", maxV);

        // TrapezoidalMotionProfiles completely ignore the acceleration when updating
        // since theres no constraint on how fast it changes
        // Just use 0 here as a placeholder
        profile.update(updateTime, updatePos, updateVel, 0);

        double totalTime = profile.totalTime();
        assertThat("Final velocity should be 0", profile.velocity(totalTime), closeTo(0, MathUtils.getFloatCompareThreshold()));
        assertThat("Final position should be as specified", profile.position(totalTime), closeTo(distance, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = helper.getDouble("updateTime", profile.totalTime());
        double updatePos = helper.getDouble("updatePos", distance);
        double updateVel = helper.getDouble("updateVel", -maxV, 0);

        // TrapezoidalMotionProfiles completely ignore the acceleration when updating
        // since theres no constraint on how fast it changes
        // Just use 0 here as a placeholder
        profile.update(updateTime, updatePos, updateVel, 0);

        double totalTime = profile.totalTime();
        assertThat("Final velocity should be 0", profile.velocity(totalTime), closeTo(0, MathUtils.getFloatCompareThreshold()));
        assertThat("Final position should be as specified", profile.position(totalTime), closeTo(distance, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = helper.getDouble("updateTime", profile.totalTime());
        double updateVel = helper.getDouble("updateVel", maxV);

        boolean overshoot = profile.update(updateTime, distance, updateVel, 0);

        assertThat("overshoot should be true", overshoot, is(true));
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
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        // Generate fake update parameters
        double updateTime = helper.getDouble("updateTime", profile.totalTime());
        double updateVel = helper.getDouble("updateVel", -maxV, 0);

        boolean overshoot = profile.update(updateTime, distance, updateVel, 0);

        assertThat("overshoot should be true", overshoot, is(true));
    }

    /**
     * Performs full testing on {@link TrapezoidalMotionProfile#copy()}.
     * 
     * This test constructs a {@link TrapezoidalMotionProfile}, and calls
     * {@link TrapezoidalMotionProfile#copy()} on it to create a copy. It then uses
     * {@link TestHelper#assertAllFieldsEqual(Object, Object)} to compare the two
     * objects for equality.
     */
    @Test
    public void testTrapezoidalMotionProfileCopy() {
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 0, 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicMotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        DynamicMotionProfile copiedProfile = profile.copy();

        TestHelper.assertAllFieldsEqual(profile, copiedProfile);
    }
}
