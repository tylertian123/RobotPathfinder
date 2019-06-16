package com.arctos6135.robotpathfinder.tests.follower;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.not;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicFollower;
import com.arctos6135.robotpathfinder.follower.DynamicTankDriveFollower;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;
import com.arctos6135.robotpathfinder.tests.follower.FollowerTest.FakeEncoder;
import com.arctos6135.robotpathfinder.tests.follower.FollowerTest.FakeGyro;
import com.arctos6135.robotpathfinder.tests.follower.FollowerTest.FakeMotor;
import com.arctos6135.robotpathfinder.tests.follower.FollowerTest.FakeTimer;

import org.junit.Test;

/**
 * This class contains tests for the dynamic followers of RobotPathfinder.
 */
public class DynamicFollowerTest {

    /**
     * A fake {@link DynamicFollowable} used for testing.
     * 
     * It returns a user-defined value for {@link DynamicFollowable#get(double)} and
     * {@link DynamicFollowable#totalTime()}, and sets a boolean to true when
     * {@link DynamicFollowable#update(Moment)} is called.
     */
    public static class FakeDynamicFollowable implements DynamicFollowable<TankDriveMoment> {

        public boolean updateCalled = false;
        public double time = 0;
        public TankDriveMoment m = new TankDriveMoment();

        @Override
        public TankDriveMoment get(double t) {
            return m;
        }

        @Override
        public double totalTime() {
            return time;
        }

        @Override
        public void update(TankDriveMoment m) {
            updateCalled = true;
        }

    }

    /**
     * Performs testing on how often {@link DynamicFollowable#update(Moment)} is
     * called.
     * 
     * This method creates a {@link DynamicTankDriveFollower} with a random update
     * delay, and asserts the following in order:
     * <ol>
     * <li>At time=0, {@code update()} is not called.</li>
     * <li>At time=updateDelay/2, {@code update()} is not called.</li>
     * <li>At time=updateDelay, {@code update()} is called.</li>
     * <li>At time=updateDelay*1.5, {@code update} is not called.</li>
     * <li>At time=updateDelay*2, {@code update} is called.</li>
     * </ol>
     */
    @Test
    public void testDynamicTankDriveFollowerUpdateTrigger() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double updateDelay = helper.getDouble("updateDelay", 1000);

        FakeDynamicFollowable followable = new FakeDynamicFollowable();

        FakeTimer timer = new FakeTimer();
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        FakeGyro gyro = new FakeGyro();

        DynamicFollower<TankDriveMoment> follower = new DynamicTankDriveFollower(followable, motor, motor, encoder,
                encoder, timer, gyro, 0, 0, 0, 0, 0, updateDelay);

        follower.initialize();
        follower.run();
        assertThat(followable.updateCalled, is(false));

        timer.value = updateDelay * 0.5;
        follower.run();
        assertThat(followable.updateCalled, is(false));

        // Set to updateDelay plus a small number so that we can make sure rounding
        // errors do not affect it
        timer.value = updateDelay + MathUtils.getFloatCompareThreshold();
        follower.run();
        assertThat(followable.updateCalled, is(true));
        followable.updateCalled = false;

        timer.value = updateDelay * 1.5;
        follower.run();
        assertThat(followable.updateCalled, is(false));

        timer.value = (updateDelay + MathUtils.getFloatCompareThreshold()) * 2;
        follower.run();
        assertThat(followable.updateCalled, is(true));
    }

    /**
     * Performs basic tests on the motor outputs of
     * {@link DynamicTankDriveFollower}.
     * 
     * This test creates a {@link DynamicTankDriveFollower} and asserts the
     * following:
     * <ul>
     * <li>With nonzero PDVA constants, time=totalTime and all sensor readings 0,
     * the motor output is non-zero.</li>
     * <li>With kV=1, kA=0, kD=0 and an arbitrary kP, and no difference between
     * position readings and the desired position, the output is equal to the
     * velocity for that time.</li>
     * <li>With kA=1, kV=0, kA=0 and an arbitary kP, and no difference between
     * position readings and the desired position, the output is equal to the
     * acceleration for that time.
     * </ul>
     */
    @Test
    public void testDynamicTankDriveFollowerMotorOutput() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1.0);
        double maxA = helper.getDouble("maxA", 1.0);
        double distance = helper.getDouble("distance", 1000);
        double kP = helper.getDouble("kP", MathUtils.getFloatCompareThreshold(), 1000);
        double kD = helper.getDouble("kD", MathUtils.getFloatCompareThreshold(), 1000);
        double kV = helper.getDouble("kV", MathUtils.getFloatCompareThreshold(), 1000);
        double kA = helper.getDouble("kA", MathUtils.getFloatCompareThreshold(), 1000);

        RobotSpecs robotSpecs = new RobotSpecs(maxV, maxA);
        TrapezoidalTankDriveProfile profile = new TrapezoidalTankDriveProfile(robotSpecs, distance);

        double checkTime = helper.getDouble("checkTime", profile.totalTime());

        FakeTimer timer = new FakeTimer();
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        Follower<TankDriveMoment> follower = new DynamicTankDriveFollower(profile, motor, motor, encoder, encoder,
                timer, kV, kA, kP, kD, profile.totalTime() * 2);

        follower.initialize();
        timer.value = profile.totalTime();
        follower.run();
        assertThat(motor.value, not(closeTo(0.0, MathUtils.getFloatCompareThreshold())));

        follower.stop();
        follower.setGains(1, 0, kP, 0);
        timer.value = 0;
        follower.initialize();
        timer.value = checkTime;
        encoder.value = profile.get(checkTime).getLeftPosition();
        follower.run();
        assertThat(motor.value,
                closeTo(profile.get(checkTime).getLeftVelocity(), MathUtils.getFloatCompareThreshold()));

        follower.stop();
        follower.setGains(0, 1, kP, 0);
        timer.value = 0;
        encoder.value = 0;
        follower.initialize();
        timer.value = checkTime;
        encoder.value = profile.get(checkTime).getLeftPosition();
        follower.run();
        assertThat(motor.value,
                closeTo(profile.get(checkTime).getLeftAcceleration(), MathUtils.getFloatCompareThreshold()));
    }
}
