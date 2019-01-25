import static org.hamcrest.Matchers.lessThan;
import static org.hamcrest.core.Is.is;
import static org.junit.Assert.assertThat;

import javax.swing.JFrame;

import org.junit.Test;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryGenerator;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.tools.Grapher;

public class TrajectoryGeneratorTest {

    @Test
    public void testGenerateStraightBasic() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 2.0);
        BasicTrajectory traj = TrajectoryGenerator.generateStraightBasic(specs, 20);

        assertThat(traj.get(traj.totalTime()).getPosition(), is(20.0));
    }

    @Test
    public void testGenerateStraightTank() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 2.0);
        TankDriveTrajectory traj = TrajectoryGenerator.generateStraightTank(specs, 20);

        assertThat(traj.get(traj.totalTime()).getLeftPosition(), is(20.0));
        assertThat(traj.get(traj.totalTime()).getRightPosition(), is(20.0));
    }

    @Test
    public void testGenerateRotationTank() {
        RobotSpecs specs = new RobotSpecs(10.0, 7.5, 1.5);
        TankDriveTrajectory traj1 = TrajectoryGenerator.generateRotationTank(specs, Math.PI / 2);
        TankDriveTrajectory traj2 = TrajectoryGenerator.generateRotationTank(specs, -Math.PI);

        assertThat(Math.abs(MathUtils.angleDiff(traj1.get(traj1.totalTime()).getFacingRelative(), MathUtils.restrictAngle(Math.PI / 2))),
                lessThan(1e-7));
        assertThat(Math.abs(MathUtils.angleDiff(traj2.get(traj2.totalTime()).getFacingRelative(), MathUtils.restrictAngle(-Math.PI))),
                lessThan(1e-7));
    }
}
