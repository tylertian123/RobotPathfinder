package robot.pathfinder.core.path;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.splinesegment.BezierSegment;
import robot.pathfinder.math.Vec2D;

/**
 * A robot path composed of multiple B&#xE9ziers. For more information, see {@link PathType}.
 * @author Tyler Tian
 *
 */
public class BezierPath extends Path {
	
	/**
	 * Constructs a path using the specified waypoints.
	 * @param waypoints The waypoints to pass through
	 * @param alpha The turn smoothness constant. A lower value will result in a relatively shorter path with sharper turns
	 * <em>at the waypoints</em>, and a higher value will result in a relatively longer path with smoother turns
	 * <em>at the waypoints</em>. However, since the turns are only smoothed near the waypoints, increasing this
	 * value too much can result in unwanted sharp turns between waypoints.
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
}
