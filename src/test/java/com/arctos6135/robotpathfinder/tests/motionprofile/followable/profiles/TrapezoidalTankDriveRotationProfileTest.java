package com.arctos6135.robotpathfinder.tests.motionprofile.followable.profiles;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveRotationProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalTankDriveRotationProfile}.
 * 
 * @author Tyler Tian
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double angle = helper.getDouble("angle", Math.PI);
        double baseWidth = helper.getDouble("baseWidth", 1000);
        double baseRadius = baseWidth / 2;

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftPosition(), closeTo(-baseRadius * angle, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getRightPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightPosition(), closeTo(baseRadius * angle, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getFacingRelative(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getFacingRelative(), closeTo(angle, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double angle = helper.getDouble("angle", -Math.PI, 0);
        double baseWidth = helper.getDouble("baseWidth", 1000);
        double baseRadius = baseWidth / 2;

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftPosition(), closeTo(-baseRadius * angle, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getRightPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightPosition(), closeTo(baseRadius * angle, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getFacingRelative(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getFacingRelative(), closeTo(angle, MathUtils.getFloatCompareThreshold()));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double angle = helper.getDouble("angle", Math.PI);
        double baseWidth = helper.getDouble("baseWidth", 1000);
        double baseRadius = baseWidth / 2;

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);

        double leftDist = -baseRadius * angle;
        double rightDist = -leftDist;
        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(),
                    either(greaterThan((leftDist))).or(closeTo((leftDist), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftPosition(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(-m.getLeftVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftVelocity(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getLeftAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightPosition(),
                    either(lessThan(rightDist)).or(closeTo(rightDist, MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightPosition(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightVelocity(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getRightAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getFacingRelative(),
                    either(lessThan(angle)).or(closeTo(angle, MathUtils.getFloatCompareThreshold())));
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
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double angle = helper.getDouble("angle", -Math.PI, 0);
        double baseWidth = helper.getDouble("baseWidth", 1000);
        double baseRadius = baseWidth / 2;

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);

        double leftDist = -baseRadius * angle;
        double rightDist = -leftDist;
        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(),
                    either(lessThan(leftDist)).or(closeTo(leftDist, MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftPosition(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getLeftVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftVelocity(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getLeftAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightPosition(),
                    either(greaterThan((rightDist))).or(closeTo((rightDist), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightPosition(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(-m.getRightVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightVelocity(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getRightAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getFacingRelative(),
                    either(greaterThan(angle)).or(closeTo(angle, MathUtils.getFloatCompareThreshold())));
        }
    }

    @Test
    public void testTrapezoidalTankDriveRotationProfileCopy() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double angle = helper.getDouble("angle", Math.PI);
        double baseWidth = helper.getDouble("baseWidth", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);

        DynamicFollowable<TankDriveMoment> f = new TrapezoidalTankDriveRotationProfile(specs, angle);

        TestHelper.assertAllFieldsEqual(f, f.copy());
    }
}
