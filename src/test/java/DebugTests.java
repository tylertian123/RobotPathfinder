

import java.io.IOException;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
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
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
        };
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
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
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
	}
}
