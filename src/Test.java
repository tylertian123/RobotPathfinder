

import java.util.ArrayList;

import javax.swing.JFrame;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.math.MathUtils;
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
				new Waypoint(-10, 25, Math.PI / 2),
				new Waypoint(0, 75, 0),
				//new Waypoint(0, 25, Math.PI / 2),
				//new Waypoint(0, 50, Math.PI / 2),
		};
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 60;
		params.segmentCount = 1000;
		params.isTank = true;
		long time = System.currentTimeMillis();
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		TankDriveTrajectory t = new TankDriveTrajectory(bt);
		long timeTaken = System.currentTimeMillis() - time;
		System.out.println("Time taken: " + timeTaken);

		JFrame f = Grapher.graphTrajectory(t, 0.01);
/*		
		//Divide and round up to get the number of samples
		int elemCount = (int) Math.ceil(bt.totalTime() / 0.01);
		
		//Create arrays to store data
		double[] time = new double[elemCount];
		double[] pos = new double[elemCount];
		double[] vel = new double[elemCount];
		double[] acl = new double[elemCount];
		
		int i = 0;
		for(double t2 = 0; t2 <= bt.totalTime(); t2 += 0.01) {
			//Collect data
			time[i] = t2;
			BasicMoment m = bt.get(t2);
			
			pos[i] = m.getPosition();
			vel[i] = m.getVelocity();
			acl[i] = m.getAcceleration();
			
			i++;
		}
		
		//Create plot
		Plot2DPanel plot = (Plot2DPanel) f.getContentPane();
		plot.setLegendOrientation("EAST");
		//Add graphs
		plot.addLinePlot("Position", time, pos);
		plot.addLinePlot("Velocity", time, vel);
		plot.addLinePlot("Acceleration", time, acl);
*/
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		JFrame f2 = Grapher.graphPath(bt.getPath(), 0.001);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void test10() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 25, Math.PI / 2),
				new Waypoint(0, 75, 0),
				//new Waypoint(0, 25, Math.PI / 2),
				//new Waypoint(0, 50, Math.PI / 2),
		};
		
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 40;
		params.segmentCount = 5000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		
		JFrame f = Grapher.graphPath(bt.getPath(), 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		
		TankDriveTrajectory tt = new TankDriveTrajectory(bt);
		
		JFrame f2 = Grapher.graphTrajectory(bt, 0.01);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
		
		BasicTrajectory mirrored = bt.retrace();
		JFrame f3 = Grapher.graphPath(mirrored.getPath(), 0.01);
		f3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f3.setVisible(true);
		
		JFrame f4 = Grapher.graphTrajectory(mirrored, 0.01);
		f4.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f4.setVisible(true);
	}
	public static void test11() {
		System.out.println(MathUtils.lerpAngle(3 * Math.PI / 4, -3 * Math.PI / 4, 0.5));
	}
	public static void test12() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 25, Math.PI / 2),
				new Waypoint(20, 0, 0),
				//new Waypoint(0, 25, Math.PI / 2),
				//new Waypoint(0, 50, Math.PI / 2),
		};
		
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 60;
		params.segmentCount = 5000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		TankDriveTrajectory tt = new TankDriveTrajectory(bt);
		
		JFrame f = Grapher.graphPath(tt.getPath(), 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		
		JFrame f2 = Grapher.graphTrajectory(tt, 0.01);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void test13() {
		RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = new Waypoint[] {
				new Waypoint(0.0, 0.0, Math.PI / 2),
				new Waypoint(-10.0, 25.0, Math.PI / 2),
				new Waypoint(0.0, 75.0, 0.0),
		};
		params.alpha = 60.0;
		params.segmentCount = 1000;
		params.roundingLimit = 1.0E-6;
		params.isTank = false;
		params.pathType = PathType.BEZIER;
		BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);
		
		JFrame f = Grapher.graphTrajectory(trajectory, 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
	}
	
	public static void main(String[] args) {
		test13();
	}
}
