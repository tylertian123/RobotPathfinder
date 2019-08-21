package com.arctos6135.robotpathfinder.tests.motionprofile.followable.profiles;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link TrapezoidalTankDriveProfile}.
 * 
 * @author Tyler Tian
 */
public class TrapezoidalTankDriveProfileTest {

    /**
     * Performs basic testing on {@link TrapezoidalTankDriveProfile}.
     * 
     * This test creates a {@link TrapezoidalTankDriveProfile} and asserts that the
     * starting position and velocity and end velocity are all 0, the end position
     * is as expected, the starting acceleration is the max acceleration, and the
     * end acceleration is negative the max acceleration for each of the sides.
     */
    @Test
    public void testTrapezoidalTankDriveProfile() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getRightPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
    }

    /**
     * Performs basic testing on {@link TrapezoidalTankDriveProfile} using a
     * negative position.
     * 
     * This test is identical to {@link #testTrapezoidalTankDriveProfile()} except
     * that it uses a negative position to construct the
     * {@link TrapezoidalTankDriveProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalTankDriveProfileReversed() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);
        TankDriveMoment begin = f.get(0);
        TankDriveMoment end = f.get(f.totalTime());
        assertThat(begin.getLeftPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getLeftAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getLeftAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));

        assertThat(begin.getRightPosition(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightVelocity(), closeTo(0.0, MathUtils.getFloatCompareThreshold()));
        assertThat(begin.getRightAcceleration(), closeTo(-maxA, MathUtils.getFloatCompareThreshold()));
        assertThat(end.getRightAcceleration(), closeTo(maxA, MathUtils.getFloatCompareThreshold()));
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalTankDriveProfile}.
     * 
     * This test creates a {@link TrapezoidalTankDriveProfile} and loops through
     * 1000 points in time, asserting that the position is between 0 and the
     * specified end position, the velocity is between 0 and the specified max
     * velocity, and the acceleration is between the negative max acceleration and
     * max acceleration for both sides.
     */
    @Test
    public void testTrapezoidalTankDriveProfileAdvanced() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(),
                    either(lessThan(distance)).or(closeTo(distance, MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftPosition(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getLeftVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftVelocity(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getLeftAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightPosition(),
                    either(lessThan(distance)).or(closeTo(distance, MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightPosition(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightVelocity(),
                    either(greaterThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getRightAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
        }
    }

    /**
     * Performs testing on many points of a {@link TrapezoidalTankDriveProfile}
     * using a negative position.
     * 
     * This test is identical to {@link #testTrapezoidalTankDriveProfileAdvanced()}
     * except that it uses a negative position to construct the
     * {@link TrapezoidalTankDriveProfile} so that it goes backwards.
     */
    @Test
    public void testTrapezoidalTankDriveProfileAdvancedReversed() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", -1000, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        Followable<TankDriveMoment> f = new TrapezoidalTankDriveProfile(specs, distance);

        double dt = f.totalTime() / 1000;
        for (double t = 0; t < f.totalTime(); t += dt) {
            TankDriveMoment m = f.get(t);
            assertThat(m.getLeftPosition(),
                    either(greaterThan((distance))).or(closeTo((distance), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftPosition(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(-m.getLeftVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getLeftVelocity(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getLeftAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));

            assertThat(m.getRightPosition(),
                    either(greaterThan((distance))).or(closeTo((distance), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightPosition(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(-m.getRightVelocity(),
                    either(lessThan((maxV))).or(closeTo((maxV), MathUtils.getFloatCompareThreshold())));
            assertThat(m.getRightVelocity(),
                    either(lessThan((0.0))).or(closeTo((0.0), MathUtils.getFloatCompareThreshold())));

            assertThat(Math.abs(m.getRightAcceleration()),
                    either(lessThan((maxA))).or(closeTo((maxA), MathUtils.getFloatCompareThreshold())));
        }
    }
}
