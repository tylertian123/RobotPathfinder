
import static org.hamcrest.Matchers.closeTo;
import static org.hamcrest.Matchers.either;
import static org.hamcrest.Matchers.greaterThan;
import static org.hamcrest.Matchers.lessThan;
import static org.junit.Assert.assertThat;

import java.io.IOException;
import java.util.Random;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.tools.Grapher;

public class DebugTests {

    public static void prompt() throws IOException {
        System.out.println("Press enter to continue execution");
        System.out.println("PID: " + ProcessHandle.current().pid());
        System.out.println("To Debug:\ngdb -p " + ProcessHandle.current().pid());
        System.in.read();
    }

    public static void test21() throws Exception {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        BasicTrajectory traj = new BasicTrajectory(specs, params);
        System.out.println(traj.totalTime());
        System.out.println(traj.get(0.01).getVelocity());
        JFrame f = Grapher.graph(traj, 0.001, true);
        traj.close();
        f.setVisible(true);
    }

    public static void test22() throws Exception {
        System.in.read();
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        var traj1 = new TankDriveTrajectory(specs, params);
        var traj = traj1.retrace();
        System.out.println(traj.totalTime());
        JFrame f1 = Grapher.graph(traj.getPath(), 0.001);
        JFrame f = Grapher.graph(traj, 0.001, true);
        traj.close();
        traj1.close();
        f.setVisible(true);
        f1.setVisible(true);
    }

    public static void main(String[] args) throws Exception {
        prompt();

        double maxV = 85.48278607323012;
        double maxA = 786.7552382177757;
        double distance = -32.83271606412042;

        System.out.println("[INFO] maxV: " + maxV);
        System.out.println("[INFO] maxA: " + maxA);
        System.out.println("[INFO] distance: " + distance);

        RobotSpecs specs = new RobotSpecs(maxV, maxA);

        MotionProfile profile = new TrapezoidalMotionProfile(specs, distance);

        double dt = profile.totalTime() / 1000;
        for (double t = 0; t < profile.totalTime(); t += dt) {
            assertThat(profile.position(t), either(greaterThan((distance))).or(closeTo((distance), 1e-7)));
            assertThat(profile.position(t), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(-profile.velocity(t), either(lessThan((maxV))).or(closeTo((maxV), 1e-7)));
            assertThat(profile.velocity(t), either(lessThan((0.0))).or(closeTo((0.0), 1e-7)));

            assertThat(Math.abs(profile.acceleration(t)), either(lessThan((maxA))).or(closeTo((maxA), 1e-7)));
        }
    }
}
