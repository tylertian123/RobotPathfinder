import static org.hamcrest.Matchers.lessThan;
import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.JNIBasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNITankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerator;
import com.arctos6135.robotpathfinder.math.MathUtils;

import org.junit.Test;

public class TrajectoryGeneratorTest {

    @Test
    public void testGenerateStraightBasic() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 2.0);
        JNIBasicTrajectory traj = TrajectoryGenerator.generateStraightBasic(specs, 20);

        assertThat(traj.get(traj.totalTime()).getPosition(), is(20.0));
        traj.free();
    }

    @Test
    public void testGenerateStraightTank() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 2.0);
        JNITankDriveTrajectory traj = TrajectoryGenerator.generateStraightTank(specs, 20);

        assertThat(traj.get(traj.totalTime()).getLeftPosition(), is(20.0));
        assertThat(traj.get(traj.totalTime()).getRightPosition(), is(20.0));
        traj.free();
    }

    @Test
    public void testGenerateRotationTank() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 1.5);
        JNITankDriveTrajectory traj1 = TrajectoryGenerator.generateRotationTank(specs, Math.PI / 2);
        JNITankDriveTrajectory traj2 = TrajectoryGenerator.generateRotationTank(specs, -Math.PI);

        assertThat(Math.abs(MathUtils.angleDiff(traj1.get(traj1.totalTime()).getFacingRelative(), MathUtils.restrictAngle(Math.PI / 2))),
                lessThan(1e-7));
        assertThat(Math.abs(MathUtils.angleDiff(traj2.get(traj2.totalTime()).getFacingRelative(), MathUtils.restrictAngle(-Math.PI))),
                lessThan(1e-7));
        traj1.free();
        traj2.free();
    }
}
