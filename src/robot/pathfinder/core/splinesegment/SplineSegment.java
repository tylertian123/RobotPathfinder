package robot.pathfinder.core.splinesegment;

import robot.pathfinder.math.Vec2D;

/**
 * An interface that defines the requirements for a spline segment.
 * <p>
 * A spline is a function defined piecewise by polynomials. Splines are what's used to fit the {@link Waypoint}s 
 * and construct paths. They're made of many segments, and each segment is defined by a parametric curve.
 * </p>
 * @author Tyler Tian
 *
 */
public interface SplineSegment {
	/**
	 * Calculates the value of the parametric curve at the specified time.
	 * @param t A value in the range [0, 1]
	 * @return The value of the parametric curve at the specified time
	 */
	public Vec2D at(double t);
	/**
	 * Calculates the derivative (tangent vector) of the parametric curve at the specified time.
	 * @param t A value in the range [0, 1]
	 * @return The value of the derivative at the specified time
	 */
	public Vec2D derivAt(double t);
	/**
	 * Calculates the second derivative of the parametric curve at the specified time.
	 * @param t A value in the range [0, 1]
	 * @return The value of the second derivative at the specified time
	 */
	public Vec2D secondDerivAt(double t);
}
