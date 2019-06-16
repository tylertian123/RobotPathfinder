package com.arctos6135.robotpathfinder.tests.follower;

import static org.junit.Assert.assertThat;
import static org.hamcrest.Matchers.is;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicFollower;
import com.arctos6135.robotpathfinder.follower.DynamicTankDriveFollower;
import com.arctos6135.robotpathfinder.math.MathUtils;
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
     * <li>At time=0, {@code update()} is not called</li>
     * <li>At time=updateDelay/2, {@code update()} is not called</li>
     * <li>At time=updateDelay, {@code update()} is called</li>
     * <li>At time=updateDelay*1.5, {@code update} is not called</li>
     * <li>At time=updateDelay*2, {@code update} is called</li>
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
}
