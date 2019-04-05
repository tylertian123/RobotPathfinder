package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;

import org.junit.Test;

public class MotionProfileTest {
    
    @Test
    public void testTrapezoidalMotionProfileBasic() {
        RobotSpecs specs = new RobotSpecs(5.0, 3.0);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, 10);
        assertThat(profile.distance(profile.totalTime()), is(10.0));
        assertThat(profile.distance(0), is(0.0));
    }
}
