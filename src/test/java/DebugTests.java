

import java.io.IOException;

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.JNIBasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNITankDriveTrajectory;
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
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;
        
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
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
        params.alpha = 20;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;
        
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

        for(int i = 0; i < 800000; i ++) {
            @SuppressWarnings("unused")
            Path path = new Path(new Waypoint[] {
                new Waypoint(0, 0, Math.PI / 2),
                new Waypoint(0, 10, Math.PI / 2),
            }, 2.0, PathType.QUINTIC_HERMITE);
        }
        System.out.println("-- do some memory intensive work --");
        for (int i = 0; i < 10; i++) {
            int[] ints = new int[1000000];
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
        System.gc();
        System.out.println("-- heavy work finished --");
	}
}
