package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.follower.BasicFollowable;
import com.arctos6135.robotpathfinder.motionprofile.followable.TrapezoidalBasicProfile;

import org.junit.Test;

public class FollowableMotionProfileTest {

    @Test
    public void testTrapezoidalBasicProfileBasic() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

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
}
