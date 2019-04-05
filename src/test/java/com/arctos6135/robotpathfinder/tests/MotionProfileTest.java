package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

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

}
