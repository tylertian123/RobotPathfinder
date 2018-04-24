package robot.pathfinder;

import robot.pathfinder.spline.QuadraticSpline;

public class QuadraticSplinePath {

	QuadraticSpline xSpline, ySpline;
	
	double scalingFactor;
	
	public QuadraticSplinePath(Waypoint... waypoints) {
		Waypoint[] xWaypoints = new Waypoint[waypoints.length];
		Waypoint[] yWaypoints = new Waypoint[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			xWaypoints[i] = new Waypoint(i, waypoints[i].getX());
			yWaypoints[i] = new Waypoint(i, waypoints[i].getY());
		}
		xSpline = new QuadraticSpline(xWaypoints);
		ySpline = new QuadraticSpline(yWaypoints);
		
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
}
