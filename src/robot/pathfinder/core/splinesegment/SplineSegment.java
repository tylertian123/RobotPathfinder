package robot.pathfinder.core.splinesegment;

import robot.pathfinder.math.Vec2D;

/**
 * 
 * @author Tyler Tian
 *
 */
public interface SplineSegment {
	public Vec2D at(double t);
	public Vec2D derivAt(double t);
	public Vec2D secondDerivAt(double t);
}
