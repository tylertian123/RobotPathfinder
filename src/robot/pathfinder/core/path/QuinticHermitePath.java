package robot.pathfinder.core.path;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.spline.QuinticHermiteSpline;
import robot.pathfinder.math.Vec2D;

public class QuinticHermitePath extends Path {
	
	double alpha;
	
	public QuinticHermitePath(Waypoint[] waypoints, double alpha) {
		this.waypoints = waypoints;
		this.alpha = alpha;
		
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Not enough waypoints");
		}
		
		segments = new QuinticHermiteSpline[waypoints.length - 1];
		for(int i = 0; i < segments.length; i ++) {
			segments[i] = new QuinticHermiteSpline(waypoints[i].asVector(), waypoints[i + 1].asVector(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha),
					Vec2D.zero, Vec2D.zero);
		}
	}
}
