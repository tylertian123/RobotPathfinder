

import java.io.IOException;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.JNIBasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNITankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerator;
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
        JNITrajectoryParams params = new JNITrajectoryParams();
        params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0, 0, Math.PI / 2),
            new JNIWaypoint(10, 10, Math.PI / 2),
        };
        params.alpha = 20;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.segmentCount = 1000;
        
        JNIBasicTrajectory traj = new JNIBasicTrajectory(specs, params);
        System.out.println(traj.totalTime());
        System.out.println(traj.get(0.01).getVelocity());
        JFrame f = Grapher.graphTrajectory(traj, 0.001, true);
        traj.close();
        f.setVisible(true);
    }

    public static void test22() throws Exception {
        System.in.read();
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        JNITrajectoryParams params = new JNITrajectoryParams();
        params.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0, 0, Math.PI / 2),
            new JNIWaypoint(10, 10, Math.PI / 2),
            new JNIWaypoint(0, 20, Math.PI),
        };
        params.alpha = 20;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.segmentCount = 1000;
        
        var traj1 = new JNITankDriveTrajectory(specs, params);
        var traj = traj1.retrace();
        System.out.println(traj.totalTime());
        JFrame f1 = Grapher.graphPath(traj.getPath(), 0.001);
        JFrame f = Grapher.graphTrajectory(traj, 0.001, true);
        traj.close();
        traj1.close();
        f.setVisible(true);
        f1.setVisible(true);
    }
	public static void main(String[] args) throws Exception {
        prompt();
        RobotSpecs specs = new RobotSpecs(5.0, 3.5, 2.0);
        var traj = TrajectoryGenerator.generateStraightTank(specs, -2 * Math.PI);
        traj.getMoments();
        //JFrame f = Grapher.graphTrajectory(traj, 0.001, true);
        //f.setVisible(true);
        traj.close();
	}
}
