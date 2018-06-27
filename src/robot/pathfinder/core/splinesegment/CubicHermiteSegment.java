package robot.pathfinder.core.splinesegment;

import robot.pathfinder.math.Vec2D;

public class CubicHermiteSegment implements SplineSegment {
	
	protected static double basis0(double t) {
		return 2 * Math.pow(t, 3) - 3 * Math.pow(t, 2) + 1;
	}
	protected static double basis1(double t) {
		return Math.pow(t, 3) - 2 * Math.pow(t, 2) + t;
	}
	protected static double basis2(double t) {
		return -2 * Math.pow(t, 3) + 3 * Math.pow(t, 2);
	}
	protected static double basis3(double t) {
		return Math.pow(t, 3) - Math.pow(t, 2);
	}
	
	protected static double basisDeriv0(double t) {
		return 6 * Math.pow(t, 2) - 6 * t;
	}
	protected static double basisDeriv1(double t) {
		return 3 * Math.pow(t, 2) - 4 * t + 1;
	}
	protected static double basisDeriv2(double t) {
		return -6 * Math.pow(t, 2) + 6 * t;
	}
	protected static double basisDeriv3(double t) {
		return 3 * Math.pow(t, 2) - 2 * t;
	}
	
	protected static double basisSecondDeriv0(double t) {
		return 12 * t - 6;
	}
	protected static double basisSecondDeriv1(double t) {
		return 6 * t - 4;
	}
	protected static double basisSecondDeriv2(double t) {
		return -12 * t + 6;
	}
	protected static double basisSecondDeriv3(double t) {
		return 6 * t - 2;
	}
	
	Vec2D p0, p1;
	Vec2D m0, m1;
	
	public CubicHermiteSegment(Vec2D p0, Vec2D p1, Vec2D m0, Vec2D m1) {
		this.p0 = p0;
		this.p1 = p1;
		this.m0 = m0;
		this.m1 = m1;
	}

	@Override
	public Vec2D at(double t) {
		return Vec2D.addVecs(p0.multiply(basis0(t)), m0.multiply(basis1(t)), p1.multiply(basis2(t)), m1.multiply(basis3(t)));
	}

	@Override
	public Vec2D derivAt(double t) {
		return Vec2D.addVecs(p0.multiply(basisDeriv0(t)), m0.multiply(basisDeriv1(t)), p1.multiply(basisDeriv2(t)), m1.multiply(basisDeriv3(t)));
	}

	@Override
	public Vec2D secondDerivAt(double t) {
		return Vec2D.addVecs(p0.multiply(basisSecondDeriv0(t)), m0.multiply(basisSecondDeriv1(t)), p1.multiply(basisSecondDeriv2(t)), m1.multiply(basisSecondDeriv3(t)));
	}

}
