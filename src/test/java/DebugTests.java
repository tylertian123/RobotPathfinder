

import javax.swing.JFrame;

import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.JNIPath;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNIBasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.JNITankDriveTrajectory;
import com.arctos6135.robotpathfinder.tools.Grapher;

public class DebugTests {

    public static void test20() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        JNITrajectoryParams jniParams = new JNITrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0.0, 0.0, Math.PI / 4),
            new Waypoint(25.0, 25.0, Math.PI / 2),
            new Waypoint(40.0, 25.0, -Math.PI / 2),
        };
        jniParams.waypoints = new JNIWaypoint[] {
            new JNIWaypoint(0.0, 0.0, Math.PI / 4),
            new JNIWaypoint(25.0, 25.0, Math.PI / 2),
            new JNIWaypoint(40.0, 25.0, -Math.PI / 2),
        };
        params.alpha = jniParams.alpha = 40.0;
        params.segmentCount = jniParams.segmentCount = 500;
        params.isTank = jniParams.isTank = true;
        params.pathType = jniParams.pathType = PathType.QUINTIC_HERMITE;
        long nanos1 = System.nanoTime();
        JNIBasicTrajectory trajectory = new JNIBasicTrajectory(robotSpecs, jniParams);
        long nanos2 = System.nanoTime();
        BasicTrajectory trajectory2 = new BasicTrajectory(robotSpecs, params);
        long nanos3 = System.nanoTime();
        trajectory.close();
        System.out.println("I AM ALIVE!");

        System.out.println("Native\t" + (nanos2 - nanos1) / 1000 + "us");
        System.out.println("Java\t" + (nanos3 - nanos2) / 1000 + "us");
    }

    public static void test21() throws Exception {
        System.out.println("Press enter to continue execution");
        System.out.println("PID: " + ProcessHandle.current().pid());
        System.in.read();
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
	public static void main(String[] args) throws Exception {
        System.out.println("Press enter to continue execution");
        System.out.println("PID: " + ProcessHandle.current().pid());
        System.in.read();
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
        
        var traj = new JNITankDriveTrajectory(specs, params);
        System.out.println(traj.totalTime());
        JFrame f = Grapher.graphTrajectory(traj, 0.001);
        traj.close();
        f.setVisible(true);
	}
}
