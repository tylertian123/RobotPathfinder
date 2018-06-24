

import java.util.ArrayList;

import javax.swing.JFrame;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.trajectory.BasicTrajectory;
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
				new Waypoint(0, 50, 0),
		};
		BasicTrajectory b = new BasicTrajectory(waypoints, 5, 3.5, 2, 20, 5000, true);

		JFrame f = Grapher.graphBasicTrajectory(b, 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		JFrame f2 = Grapher.graphPath(b.getPath(), 0.005);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void main(String[] args) {
		test9();
	}
}
