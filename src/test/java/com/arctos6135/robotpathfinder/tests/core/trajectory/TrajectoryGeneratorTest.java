package com.arctos6135.robotpathfinder.tests.core.trajectory;

import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerator;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.tests.TestHelper;

import org.junit.Test;

/**
 * This class contains tests for {@link TrajectoryGenerator}.
 * 
 * Since {@link TrajectoryGenerator} is now deprecated, this class may also be
 * removed in the future.
 * 
 * @author Tyler Tian
 */
@SuppressWarnings("deprecation")
public class TrajectoryGeneratorTest {

    /**
     * Performs basic testing on
     * {@link TrajectoryGenerator#generateStraightBasic(RobotSpecs, double)}.
     * 
     * This test asserts that the position at the end of the generated trajectory is
     * as expected.
     */
    @Test
    public void testGenerateStraightBasic() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);
        double baseWidth = helper.getDouble("baseWidth", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);
        BasicTrajectory traj = TrajectoryGenerator.generateStraightBasic(specs, distance);

        assertThat(traj.get(traj.totalTime()).getPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        traj.close();
    }

    /**
     * Performs basic testing on
     * {@link TrajectoryGenerator#generateStraightTank(RobotSpecs, double)}.
     * 
     * This test asserts that the position at the end of the generated trajectory is
     * as expected.
     */
    @Test
    public void testGenerateStraightTank() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double distance = helper.getDouble("distance", 1000);
        double baseWidth = helper.getDouble("baseWidth", 1000);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);
        TankDriveTrajectory traj = TrajectoryGenerator.generateStraightTank(specs, distance);

        assertThat(traj.get(traj.totalTime()).getLeftPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        assertThat(traj.get(traj.totalTime()).getRightPosition(), closeTo(distance, MathUtils.getFloatCompareThreshold()));
        traj.close();
    }

    /**
     * Performs basic testing on
     * {@link TrajectoryGenerator#generateRotationTank(RobotSpecs, double)}.
     * 
     * This test asserts that the angle (relative facing) at the end of the
     * generated trajectory is as expected.
     */
    @Test
    public void testGenerateRotationTank() {
        TestHelper helper = TestHelper.getInstance(getClass());

        double maxV = helper.getDouble("maxV", 1000);
        double maxA = helper.getDouble("maxA", 1000);
        double baseWidth = helper.getDouble("baseWidth", 1000);
        double angle1 = helper.getDouble("angle1", Math.PI / 2);
        double angle2 = helper.getDouble("angle2", -Math.PI / 2, 0);

        RobotSpecs specs = new RobotSpecs(maxV, maxA, baseWidth);
        TankDriveTrajectory traj1 = TrajectoryGenerator.generateRotationTank(specs, angle1);
        TankDriveTrajectory traj2 = TrajectoryGenerator.generateRotationTank(specs, angle2);

        assertThat(Math.abs(MathUtils.angleDiff(traj1.get(traj1.totalTime()).getFacingRelative(),
                MathUtils.restrictAngle(angle1))), lessThan(MathUtils.getFloatCompareThreshold()));
        assertThat(Math.abs(MathUtils.angleDiff(traj2.get(traj2.totalTime()).getFacingRelative(),
                MathUtils.restrictAngle(angle2))), lessThan(MathUtils.getFloatCompareThreshold()));
        traj1.close();
        traj2.close();
    }
}
