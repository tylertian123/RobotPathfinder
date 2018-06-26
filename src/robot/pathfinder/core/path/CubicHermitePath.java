package robot.pathfinder.core.path;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.spline.CubicHermiteSpline;
import robot.pathfinder.math.Vec2D;

public class CubicHermitePath extends Path {

	double alpha;
	
	public CubicHermitePath(Waypoint[] waypoints, double alpha) {
		this.waypoints = waypoints;
		this.alpha = alpha;
		
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Not enough waypoints");
		}
		
		segments = new CubicHermiteSpline[waypoints.length - 1];
		for(int i = 0; i < segments.length; i ++) {
			segments[i] = new CubicHermiteSpline(waypoints[i].asVector(), waypoints[i + 1].asVector(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha));
		}
	}
}
