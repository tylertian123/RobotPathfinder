package robot.pathfinder.core.splinesegment;

import robot.pathfinder.math.Vec2D;

/**
 * A class representing a cubic B&#xE9zier curve. Can be used as a {@link SplineSegment}.
 * @author Tyler Tian
 *
 */
public class BezierSegment implements SplineSegment {
	Vec2D[] controlPoints;
	
	/**
	 * Constructs a new cubic B&#xE9zier with the specified control points.
	 * @param a The first control point
	 * @param b The second control point
	 * @param c The third control point
	 * @param d The last control point
	 */
	public BezierSegment(Vec2D a, Vec2D b, Vec2D c, Vec2D d) {
		controlPoints = new Vec2D[4];
		controlPoints[0] = a;
		controlPoints[1] = b;
		controlPoints[2] = c;
		controlPoints[3] = d;
	}
	
	/**
	 * Returns a new cubic B&#xE9zier with the specified start and end points, and derivatives at those points.
	 * @param at0 The starting control point
	 * @param at1 The ending control point
	 * @param derivAt0 The derivative at the starting control point
	 * @param derivAt1 The derivative at the ending control point
	 * @return A new {@link BezierSegment} instance that follows the constraints
	 */
	public static BezierSegment getFromHermite(Vec2D at0, Vec2D at1, Vec2D derivAt0, Vec2D derivAt1) {
		Vec2D p1 = at0.add(derivAt0.multiply(1.0 / 3.0));
		Vec2D p2 = at1.add(derivAt1.multiply(-1.0 / 3.0));
		return new BezierSegment(at0, p1, p2, at1);
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vec2D at(double t) {
		double u = 1 - t;
		double uu = u * u;
		double uuu = u * u * u;
		double tt = t * t;
		double ttt = t * t * t;
		return Vec2D.addVecs(controlPoints[0].multiply(uuu), controlPoints[1].multiply(3 * uu * t),
				controlPoints[2].multiply(3 * u * tt), controlPoints[3].multiply(ttt));
	}
	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vec2D derivAt(double t) {
		double u = 1 - t;
		double uu = u * u;
		double tt = t * t;
		return Vec2D.addVecs(controlPoints[1].add(controlPoints[0].multiply(-1)).multiply(3 * uu),
				controlPoints[2].add(controlPoints[1].multiply(-1)).multiply(6 * u * t),
				controlPoints[3].add(controlPoints[2].multiply(-1)).multiply(3 * tt));
	}
	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vec2D secondDerivAt(double t) {
		double u = 1 - t;
		return Vec2D.addVecs(Vec2D.addVecs(controlPoints[2], controlPoints[1].multiply(-2), controlPoints[0]).multiply(6 * u),
				Vec2D.addVecs(controlPoints[3], controlPoints[2].multiply(-2), controlPoints[1]).multiply(6 * t));
	}
}
