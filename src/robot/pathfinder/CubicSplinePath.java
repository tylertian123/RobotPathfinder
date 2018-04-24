package robot.pathfinder;

import java.util.ArrayList;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.spline.CubicSpline;

public class CubicSplinePath {
	
	CubicSpline xSpline, ySpline;
	
	double scalingFactor;
	
	public CubicSplinePath(Waypoint... waypoints) {
		Waypoint[] xWaypoints = new Waypoint[waypoints.length];
		Waypoint[] yWaypoints = new Waypoint[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			xWaypoints[i] = new Waypoint(i, waypoints[i].getX());
			yWaypoints[i] = new Waypoint(i, waypoints[i].getY());
		}
		xSpline = new CubicSpline(xWaypoints);
		ySpline = new CubicSpline(yWaypoints);
		
		scalingFactor = waypoints.length - 1;
	}
	
	public Waypoint at(double t) {
		t *= scalingFactor;
		return new Waypoint(xSpline.at(t), ySpline.at(t));
	}
	public Waypoint derivAt(double t) {
		t *= scalingFactor;
		return new Waypoint(xSpline.derivAt(t), ySpline.derivAt(t));
	}
	public Waypoint secondDerivAt(double t) {
		t *= scalingFactor;
		return new Waypoint(xSpline.secondDerivAt(t), ySpline.secondDerivAt(t));
	}
	
	
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	public static void main(String[] args) {
		final double t_delta = 0.001;
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0),
				new Waypoint(0, 5),
				new Waypoint(-5, 20),
				new Waypoint(-5, 22.5),
				new Waypoint(-5, 25),
		};
		
		CubicSplinePath csp = new CubicSplinePath(waypoints);
		for(double t = 0; t <= 1; t += t_delta) {
			Waypoint w = csp.at(t);
			xs.add(w.getX());
			ys.add(w.getY());
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("Position", primitiveArr(xs), primitiveArr(ys));
		double[] waypointX = new double[waypoints.length];
		double[] waypointY = new double[waypoints.length];
		for(int i = 0; i < waypoints.length; i ++) {
			waypointX[i] = waypoints[i].getX();
			waypointY[i] = waypoints[i].getY();
		}
		plot.addScatterPlot("Waypoints", waypointX, waypointY);
		
		JFrame frame = new JFrame();
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		frame.setVisible(true);
	}
}
