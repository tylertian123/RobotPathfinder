package robot.pathfinder.test;

import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.Moment;
import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.tools.Grapher;

public class Testing {
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	
	public static void test4() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-20, 20, Math.PI / 2),
		};
		long time = System.currentTimeMillis();
		TankDriveTrajectory.setSolverRoundingLimit(0);
		TankDriveTrajectory b = new TankDriveTrajectory(waypoints, 5, 3.5, 1.0, 2, 20, 5000, true);
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		
		double testTime = b.totalTime() / 2;
		long nanos = System.nanoTime();
		@SuppressWarnings("unused")
		Moment left = b.getLeftSmooth(testTime);
		@SuppressWarnings("unused")
		Moment right = b.getRightSmooth(testTime);
		long nanosTime = System.nanoTime() - nanos;
		System.out.println("Getting one pair of values took " + nanosTime + " nanoseconds");
		
		JFrame pGraph = Grapher.graphPath(b.getPath(), 0.005);
		pGraph.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		pGraph.setVisible(true);
		
		JFrame tGraph = Grapher.graphTrajectory(b, 0.01);
		tGraph.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		tGraph.setVisible(true);
	}
	
	public static void test5() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(0, 100, Math.PI / 2),
		};
		TankDriveTrajectory b = new TankDriveTrajectory(waypoints, 5, 3.5, 1.0, 2, 20, 5000, true);

		double dt = b.totalTime() / 5000;
		long start = System.nanoTime();
		for(double t = 0; t < b.totalTime(); t += dt) {
			b.getLeftSmooth(t);
		}
		long time = System.nanoTime() - start;
		System.out.println(time);
	}
	
	public static void test6() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-5, 10, 3 * Math.PI / 4),
				new Waypoint(-20, 20, Math.PI / 2),
		};
		long time = System.currentTimeMillis();
		TankDriveTrajectory.setSolverRoundingLimit(0);
		TankDriveTrajectory a = new TankDriveTrajectory(waypoints, 5, 3.5, 1.0, 2, 20, 5000, true);
		TankDriveTrajectory b = a.reverse();
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		
		JFrame pGraph = Grapher.graphTrajectory(a, 0.005);
		pGraph.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		pGraph.setVisible(true);
		JFrame pGraph2 = Grapher.graphTrajectory(b, 0.005);
		pGraph2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		pGraph2.setVisible(true);
	}
	
	public static void main(String[] args) {
		test6();
	}
}
