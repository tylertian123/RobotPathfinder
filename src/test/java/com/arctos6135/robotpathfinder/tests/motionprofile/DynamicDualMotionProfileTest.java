package com.arctos6135.robotpathfinder.tests.motionprofile;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.motionprofile.DynamicDualMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.DynamicMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

/**
 * This class contains tests for {@link DynamicDualMotionProfile}.
 * 
 * @author Tyler Tian
 */
public class DynamicDualMotionProfileTest {

    @Rule
    public TestName testName = new TestName();

    /**
     * Performs full testing on {@link DynamicDualMotionProfile#copy()}.
     * 
     * This test constructs an instance, and calls the copy method on it to create a
     * copy. It then uses {@link TestHelper#assertAllFieldsEqual(Object, Object)} to
     * compare the two objects for equality.
     */
    @Test
    public void testDynamicDualMotionProfileCopy() {
        TestHelper helper = new TestHelper(getClass(), testName);

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double ldistance = helper.getDouble("ldistance", 0, 1000);
        double rdistance = helper.getDouble("rdistance", 0, 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);
        DynamicMotionProfile lProfile = new TrapezoidalMotionProfile(specs, ldistance);
        DynamicMotionProfile rProfile = new TrapezoidalMotionProfile(specs, rdistance);

        DynamicDualMotionProfile<DynamicMotionProfile> profile = new DynamicDualMotionProfile<DynamicMotionProfile>(
                lProfile, rProfile);
        DynamicDualMotionProfile<DynamicMotionProfile> copiedProfile = profile.copy();

        TestHelper.assertAllFieldsEqual(profile, copiedProfile);
    }
}
