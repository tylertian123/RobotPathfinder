

import javax.swing.JFrame;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryGenerator;
import robot.pathfinder.tools.Grapher;

public class DebugTests {
    
    public static void test18() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
                new WaypointEx(0.0, 0.0, Math.PI / 4, 2.0),
                new WaypointEx(25.0, 25.0, Math.PI / 2, 0),
                new WaypointEx(40.0, 25.0, -Math.PI / 2, 2.0),
        };
        params.alpha = 40.0;
        params.segmentCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        JFrame f1 = Grapher.graphPath(trajectory.getPath(), 0.01);
        f1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f1.setVisible(true);
        JFrame f2 = Grapher.graphTrajectory(trajectory, 0.01);
        f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f2.setVisible(true);
    }
    
    public static void test19() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 3.0);

        TankDriveTrajectory traj = TrajectoryGenerator.generateRotationTank(robotSpecs, -Math.PI);

        JFrame f1 = Grapher.graphTrajectory(traj, 0.001, true);
        f1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f1.setVisible(true);
    }

	public static void main(String[] args) throws Exception {
		RobotSpecs specs = new RobotSpecs(10.0, 7.5, 1.5);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
        };
        params.alpha = 20;
        params.isTank = true;
        
        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params).retrace();
        JFrame f = Grapher.graphTrajectory(traj, 0.01, true);
        //f.setVisible(true);
        JFrame f2 = Grapher.graphPath(traj.getPath(), 0.01);
        //f2.setVisible(true);

        System.out.println(traj.get(0).getInitialFacing());
        System.out.println(traj.get(0).getBackwards());
	}
}
