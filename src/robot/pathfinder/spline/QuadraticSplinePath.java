package robot.pathfinder.spline;

import robot.pathfinder.Vec2D;
import robot.pathfinder.Waypoint;

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
	
	public Vec2D at(double t) {
		t *= scalingFactor;
		return new Vec2D(xSpline.at(t), ySpline.at(t));
	}
	public Vec2D derivAt(double t) {
		t *= scalingFactor;
		return new Vec2D(xSpline.derivAt(t), ySpline.derivAt(t));
	}
	public Vec2D secondDerivAt(double t) {
		t *= scalingFactor;
		return new Vec2D(xSpline.secondDerivAt(t), ySpline.secondDerivAt(t));
	}
}
