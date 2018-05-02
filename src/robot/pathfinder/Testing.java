package robot.pathfinder;

import java.util.ArrayList;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.old.Moment;

public class Testing {
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	public static void test1() {
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		ArrayList<Double> leftxs = new ArrayList<Double>();
		ArrayList<Double> leftys = new ArrayList<Double>();
		ArrayList<Double> rightxs = new ArrayList<Double>();
		ArrayList<Double> rightys = new ArrayList<Double>();
		ArrayList<Double> ts = new ArrayList<Double>();
		ArrayList<Double> xvs = new ArrayList<Double>();
		ArrayList<Double> yvs = new ArrayList<Double>();
		ArrayList<Double> xas = new ArrayList<Double>();
		ArrayList<Double> yas = new ArrayList<Double>();
		
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-5, 4, Math.PI / 2),
				new Waypoint(-5, 7, Math.PI / 2),
				new Waypoint(0, 10, Math.PI / 2),
				new Waypoint(10, 10, Math.PI / 2),
		};
		
		RobotPath path = new RobotPath(10, waypoints, 0.5);
		for(double t = 0; t <= 1; t += t_delta) {
			Vec2D w = path.at(t);
			xs.add(w.getX());
			ys.add(w.getY());
			ts.add(t);
			Vec2D l = path.leftWheelAt(t);
			leftxs.add(l.getX());
			leftys.add(l.getY());
			Vec2D r = path.rightWheelAt(t);
			rightxs.add(r.getX());
			rightys.add(r.getY());
			Vec2D v = path.derivAt(t);
			Vec2D a = path.secondDerivAt(t);
			xvs.add(v.getX());
			yvs.add(v.getY());
			xas.add(a.getX());
			yas.add(a.getY());
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Position", primitiveArr(xs), primitiveArr(ys));
		plot.addLinePlot("Left Wheel", primitiveArr(leftxs), primitiveArr(leftys));
		plot.addLinePlot("Right Wheel", primitiveArr(rightxs), primitiveArr(rightys));
		double[] waypointX = new double[waypoints.length];
		double[] waypointY = new double[waypoints.length];
		for(int i = 0; i < waypoints.length; i ++) {
			waypointX[i] = waypoints[i].getX();
			waypointY[i] = waypoints[i].getY();
		}
		plot.addScatterPlot("Waypoints", waypointX, waypointY);
		plot.setFixedBounds(0, -25, 25);
		plot.setFixedBounds(1, 0, 50);
		plot.setLegendOrientation("EAST");
		
		JFrame frame = new JFrame("Position");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);
		
		double[] time = primitiveArr(ts);
		Plot2DPanel plot2 = new Plot2DPanel();
		plot2.addLinePlot("X Position", time, primitiveArr(xs));
		plot2.addLinePlot("Y Position", time, primitiveArr(ys));
		plot2.addLinePlot("X Velocity", time, primitiveArr(xvs));
		plot2.addLinePlot("Y Velocity", time, primitiveArr(yvs));
		plot2.addLinePlot("X Acceleration", time, primitiveArr(xas));
		plot2.addLinePlot("Y Acceleration", time, primitiveArr(yas));
		plot2.setLegendOrientation("EAST");
		
		JFrame frame2 = new JFrame("XY graph");
		frame2.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame2.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame2.setContentPane(plot2);
		frame2.setVisible(true);
	}
	public static void test2() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 10, Math.PI / 2),
				new Waypoint(10, 20, Math.PI / 2),
				new Waypoint(5, 10, -Math.PI / 2),
		};
		BezierPath b = new BezierPath(waypoints, 10);
		b.setBaseRadius(1);
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		
		for(double t = 0; t <= 1; t += t_delta) {
			Vec2D v = b.at(t);
			xs.add(v.getX());
			ys.add(v.getY());
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Bezier", primitiveArr(xs), primitiveArr(ys));
		JFrame frame = new JFrame();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);
	}
	public static void test3() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 10, Math.PI / 2),
				new Waypoint(10, 20, Math.PI / 2),
				new Waypoint(5, 10, -Math.PI / 2),
		};
		long time = System.currentTimeMillis();
		BezierTrajectory b = new BezierTrajectory(waypoints, 10, 3, 2, 2, 1000);
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		ArrayList<Double> ds = new ArrayList<Double>();
		ArrayList<Double> vs = new ArrayList<Double>();
		ArrayList<Double> as = new ArrayList<Double>();
		ArrayList<Double> ts = new ArrayList<Double>();
		ArrayList<Double> mvs = new ArrayList<Double>();
		
		for(double t = 0; t <= 1; t += t_delta) {
			Vec2D v = b.pathAt(t);
			xs.add(v.getX());
			ys.add(v.getY());
		}
		for(double t = 0; t <= b.totalTime(); t += 0.005) {
			Moment m = b.getMoment(t);
			ds.add(m.getDist());
			vs.add(m.getVelo());
			as.add(m.getAccel());
			ts.add(t);
			mvs.add(m.getMaxVelo());
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Bezier", primitiveArr(xs), primitiveArr(ys));
		JFrame frame = new JFrame("Bezier");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);
		
		Plot2DPanel plot2 = new Plot2DPanel();
		plot2.addLinePlot("Position", primitiveArr(ts), primitiveArr(ds));
		plot2.addLinePlot("Velocity", primitiveArr(ts), primitiveArr(vs));
		plot2.addLinePlot("Acceleration", primitiveArr(ts), primitiveArr(as));
		plot2.addLinePlot("Theoretical Max Velocity", primitiveArr(ts), primitiveArr(mvs));
		plot2.setLegendOrientation("EAST");
		JFrame frame2 = new JFrame("PVA");
		frame2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame2.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame2.setContentPane(plot2);
		frame2.setVisible(true);
	}
	public static void test4() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 10, Math.PI / 2),
				new Waypoint(10, 20, Math.PI / 2),
				new Waypoint(5, 10, -Math.PI / 2),
		};
		long time = System.currentTimeMillis();
		TankDriveTrajectory b = new TankDriveTrajectory(waypoints, 3, 2, 2, 10, 1000);
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		
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
		}
		System.out.println(b.totalTime());
		for(double t = 0; t <= b.totalTime(); t += 0.005) {
			robot.pathfinder.Moment left = b.getLeft(t);
			robot.pathfinder.Moment right = b.getRight(t);
			lds.add(left.getDistance());
			rds.add(right.getDistance());
			lvs.add(left.getVelocity());
			rvs.add(right.getVelocity());
			las.add(left.getAcceleration());
			ras.add(right.getAcceleration());
			ts.add(t);
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Bezier", primitiveArr(xs), primitiveArr(ys));
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
