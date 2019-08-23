package com.arctos6135.robotpathfinder.tests.motionprofile.followable.profiles;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalBasicProfile}.
 * 
 * @author Tyler Tian
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);
        BasicMoment begin = f.get(0);
        BasicMoment end = f.get(f.totalTime());
        assertThat(begin.getPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            BasicMoment m = f.get(t);
            assertThat(m.getPosition(),
                    either(lessThan(distance)).or(closeTo(distance, MathUtils.getFloatCompareThreshold())));
            assertThat(m.getPosition(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getVelocity(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            BasicMoment m = f.get(t);
            assertThat(m.getPosition(),
                    either(greaterThan((distance))).or(closeTo((distance), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getPosition(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(-m.getVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getVelocity(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
        }
    }

    /**
     * Performs full testing on {@link TrapezoidalBasicProfile#copy()}.
     * 
     * This test constructs an instance, and calls the copy method on it to create a
     * copy. It then uses {@link TestHelper#assertAllFieldsEqual(Object, Object)} to
     * compare the two objects for equality.
     */
    @Test
    public void testTrapezoidalBasicProfileCopy() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        DynamicFollowable<BasicMoment> f = new TrapezoidalBasicProfile(specs, distance);

        TestHelper.assertAllFieldsEqual(f, f.copy());
    }
}
