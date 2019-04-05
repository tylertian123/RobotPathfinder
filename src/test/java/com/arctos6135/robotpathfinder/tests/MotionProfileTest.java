package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;
import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;

import java.util.Random;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;

import org.junit.Test;

public class MotionProfileTest {
    
    @Test
    public void testTrapezoidalMotionProfileBasic() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat(profile.distance(profile.totalTime()), is(distance));
        assertThat(profile.distance(0), is(0.0));
        assertThat(profile.velocity(0), is(0.0));
        assertThat(profile.velocity(profile.totalTime()), is(0.0));
        assertThat(profile.acceleration(0), is(maxA));
        assertThat(profile.acceleration(profile.totalTime()), is(-maxA));
    }

    @Test
    public void testTrapezoidalMotionProfileBasicReversed() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = -rand.nextDouble() * 1000;

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);
        assertThat(profile.distance(profile.totalTime()), is(distance));
        assertThat(Math.abs(profile.distance(0)), is(0.0));
        assertThat(Math.abs(profile.velocity(0)), is(0.0));
        assertThat(Math.abs(profile.velocity(profile.totalTime())), is(0.0));
        assertThat(profile.acceleration(0), closeTo(-maxA, 1e-7));
        assertThat(profile.acceleration(profile.totalTime()), closeTo(maxA, 1e-7));
    }

    @Test
    public void testTrapezoidalMotionProfileAdvanced() {
        Random rand = new Random();
        double maxV = rand.nextDouble() * 1000;
        double maxA = rand.nextDouble() * 1000;
        double distance = rand.nextDouble() * 1000;

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for(double t = 0; t < profile.totalTime(); t += dt) {
            assertThat(profile.distance(t), is(lessThanOrEqualTo(distance)));
            assertThat(profile.distance(t), is(greaterThanOrEqualTo(0.0)));

            assertThat(profile.velocity(t), is(lessThanOrEqualTo(maxV)));
            assertThat(profile.velocity(t), is(greaterThanOrEqualTo(0.0)));

            assertThat(Math.abs(profile.acceleration(t)), is(lessThanOrEqualTo(maxA)));
        }
    }
}
