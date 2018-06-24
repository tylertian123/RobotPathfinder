

import java.util.ArrayList;

import javax.swing.JFrame;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryParams;
import robot.pathfinder.tankdrive.TankDriveTrajectory;
import robot.pathfinder.tools.Grapher;

public class Test {
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	
	public static void test9() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 25, Math.PI / 4),
				new Waypoint(0, 50, 0),
		};
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 40;
		params.segmentCount = 10000;
		params.isTank = true;
		
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		TankDriveTrajectory t = new TankDriveTrajectory(bt);

		JFrame f = Grapher.graphTrajectory(t, 0.05);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		JFrame f2 = Grapher.graphPath(t.getPath(), 0.01);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void main(String[] args) {
		test9();
	}
}
