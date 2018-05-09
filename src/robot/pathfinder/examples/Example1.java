package robot.pathfinder.examples;

import java.util.ArrayList;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.Moment;
import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;

public class Example1 {
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
				new Waypoint(-10, 50, 0),
		};
		long time = System.currentTimeMillis();
		TankDriveTrajectory.setSolverRoundingLimit(0);
		TankDriveTrajectory b = new TankDriveTrajectory(waypoints, 5, 3.5, 2, 20, 5000, true);
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		ArrayList<Double> lx = new ArrayList<Double>();
		ArrayList<Double> ly = new ArrayList<Double>();
		ArrayList<Double> rx = new ArrayList<Double>();
		ArrayList<Double> ry = new ArrayList<Double>();
		
		ArrayList<Double> lds = new ArrayList<Double>();
		ArrayList<Double> lvs = new ArrayList<Double>();
		ArrayList<Double> las = new ArrayList<Double>();
		ArrayList<Double> rds = new ArrayList<Double>();
		ArrayList<Double> rvs = new ArrayList<Double>();
		ArrayList<Double> ras = new ArrayList<Double>();
		ArrayList<Double> ts = new ArrayList<Double>();
		
		for(double t = 0; t <= 1; t += t_delta) {
			Vec2D v = b.pathAt(t);
			xs.add(v.getX());
			ys.add(v.getY());
			Vec2D[] wheels = b.getPath().wheelsAt(t);
			lx.add(wheels[0].getX());
			ly.add(wheels[0].getY());
			rx.add(wheels[1].getX());
			ry.add(wheels[1].getY());
		}
		System.out.println(b.totalTime());
		for(double t = 0; t <= b.totalTime(); t += 0.005) {
			Moment left = b.getLeftSmooth(t);
			Moment right = b.getRightSmooth(t);
			lds.add(left.getDistance());
			rds.add(right.getDistance());
			lvs.add(left.getVelocity());
			rvs.add(right.getVelocity());
			las.add(left.getAcceleration());
			ras.add(right.getAcceleration());
			ts.add(t);
		}
		double testTime = b.totalTime() / 2;
		long nanos = System.nanoTime();
		@SuppressWarnings("unused")
		Moment left = b.getLeftSmooth(testTime);
		@SuppressWarnings("unused")
		Moment right = b.getRightSmooth(testTime);
		long nanosTime = System.nanoTime() - nanos;
		System.out.println("Getting one pair of values took " + nanosTime + " nanoseconds");
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Bezier", primitiveArr(xs), primitiveArr(ys));
		plot.addLinePlot("Left", primitiveArr(lx), primitiveArr(ly));
		plot.addLinePlot("Right", primitiveArr(rx), primitiveArr(ry));
		plot.setLegendOrientation("EAST");
		JFrame frame = new JFrame("Bezier");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);
		
		Plot2DPanel plot2 = new Plot2DPanel();
		plot2.addLinePlot("Left Position", primitiveArr(ts), primitiveArr(lds));
		plot2.addLinePlot("Left Velocity", primitiveArr(ts), primitiveArr(lvs));
		plot2.addLinePlot("Left Acceleration", primitiveArr(ts), primitiveArr(las));
		plot2.addLinePlot("Right Position", primitiveArr(ts), primitiveArr(rds));
		plot2.addLinePlot("Right Velocity", primitiveArr(ts), primitiveArr(rvs));
		plot2.addLinePlot("Right Acceleration", primitiveArr(ts), primitiveArr(ras));
		plot2.setLegendOrientation("EAST");
		JFrame frame2 = new JFrame("PVA");
		frame2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame2.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame2.setContentPane(plot2);
		frame2.setVisible(true);
	}
	public static void main(String[] args) {
		test4();
	}
}
