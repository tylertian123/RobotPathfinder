package robot.pathfinder.core.path;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.splinesegment.BezierSegment;
import robot.pathfinder.math.Vec2D;

/**
 * A robot path composed of multiple Beziers.
 * @author Tyler Tian
 *
 */
public class BezierPath extends Path {
	
	/**
	 * Constructs a BezierSegment path using the specified waypoints.
	 * @param waypoints The waypoints to pass through
	 * @param alpha The smoothness of the turns; a greater value will result in smoother turns and a small value will result in tight turns.
	 */
	public BezierPath(Waypoint[] waypoints, double alpha) {
		type = PathType.BEZIER;
		
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Not enough waypoints");
		}
		segments = new BezierSegment[waypoints.length - 1];
		for(int i = 0; i < segments.length; i ++) {
			segments[i] = BezierSegment.getFromHermite(waypoints[i].asVector(), waypoints[i + 1].asVector(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha));
		}
		
		this.waypoints = waypoints;
		this.alpha = alpha;
	}
	public BezierPath(Waypoint[] waypoints, double alpha, double baseRadius) {
		this(waypoints, alpha);
		this.baseRadius = baseRadius;
	}
}
