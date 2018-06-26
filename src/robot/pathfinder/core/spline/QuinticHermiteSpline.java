package robot.pathfinder.core.spline;

import robot.pathfinder.math.Vec2D;

public class QuinticHermiteSpline implements Spline {
	
	protected static double basis0(double t) {
		return 1 - 10 * Math.pow(t, 3) + 15 * Math.pow(t, 4) - 6 * Math.pow(t, 5);
	}
	protected static double basis1(double t) {
		return t - 6 * Math.pow(t, 3) + 8 * Math.pow(t, 4) - 3 * Math.pow(t, 5);
	}
	protected static double basis2(double t) {
		return Math.pow(t, 2) / 2 - 3 * Math.pow(t, 3) / 2 + 3 * Math.pow(t, 4) / 2 - Math.pow(t, 5) / 2;
	}
	protected static double basis3(double t) {
		return Math.pow(t, 3) / 2 - Math.pow(t, 4) + Math.pow(t, 5) / 2;
	}
	protected static double basis4(double t) {
		return -4 * Math.pow(t, 3) + 7 * Math.pow(t, 4) - 3 * Math.pow(t, 5);
	}
	protected static double basis5(double t) {
		return 10 * Math.pow(t, 3) - 15 * Math.pow(t, 4) + 6 * Math.pow(t, 5);
	}
	
	protected static double basisDeriv0(double t) {
		return -30 * Math.pow(t, 2) + 60 * Math.pow(t, 3) - 30 * Math.pow(t, 4);
	}
	protected static double basisDeriv1(double t) {
		return 1 - 18 * Math.pow(t, 2) + 32 * Math.pow(t, 3) - 15 * Math.pow(t, 4);
	}
	protected static double basisDeriv2(double t) {
		return t - 9 * Math.pow(t, 2) / 2 + 6 * Math.pow(t, 3) - 5 * Math.pow(t, 4) / 2;
	}
	protected static double basisDeriv3(double t) {
		return 3 * Math.pow(t, 2) / 2 - 4 * Math.pow(t, 3) + 5 * Math.pow(t,  4) / 2;
	}
	protected static double basisDeriv4(double t) {
		return -12 * Math.pow(t, 2) + 28 * Math.pow(t, 3) - 15 * Math.pow(t, 4);
	}
	protected static double basisDeriv5(double t) {
		return 30 * Math.pow(t, 2) - 60 * Math.pow(t, 3) + 30 * Math.pow(t, 4);
	}
	
	protected static double basisSecondDeriv0(double t) {
		return -60 * t + 180 * Math.pow(t, 2) - 120 * Math.pow(t, 3);
	}
	protected static double basisSecondDeriv1(double t) {
		return -36 * t + 96 * Math.pow(t, 2) - 60 * Math.pow(t, 3);
	}
	protected static double basisSecondDeriv2(double t) {
		return 1 - 9 * t + 18 * Math.pow(t, 2) - 10 * Math.pow(t, 3);
	}
	protected static double basisSecondDeriv3(double t) {
		return 3 * t - 12 * Math.pow(t, 2) + 10 * Math.pow(t, 3);
	}
	protected static double basisSecondDeriv4(double t) {
		return -24 * t + 84 * Math.pow(t, 2) - 60 * Math.pow(t, 3);
	}
	protected static double basisSecondDeriv5(double t) {
		return 60 * t - 180 * Math.pow(t, 2) + 120 * Math.pow(t, 3);
	}

	Vec2D p0, p1;
	Vec2D v0, v1;
	Vec2D a0, a1;
	
	public QuinticHermiteSpline(Vec2D p0, Vec2D p1, Vec2D v0, Vec2D v1, Vec2D a0, Vec2D a1) {
		this.p0 = p0;
		this.p1 = p1;
		this.v0 = v0;
		this.v1 = v1;
		this.a0 = a0;
		this.a1 = a1;
	}
	
	@Override
	public Vec2D at(double t) {
		return Vec2D.addVecs(p0.multiply(basis0(t)), v0.multiply(basis1(t)), a0.multiply(basis2(t)),
				a1.multiply(basis3(t)), v1.multiply(basis4(t)), p1.multiply(basis5(t)));
	}
	@Override
	public Vec2D derivAt(double t) {
		return Vec2D.addVecs(p0.multiply(basisDeriv0(t)), v0.multiply(basisDeriv1(t)), a0.multiply(basisDeriv2(t)),
				a1.multiply(basisDeriv3(t)), v1.multiply(basisDeriv4(t)), p1.multiply(basisDeriv5(t)));
	}
	@Override
	public Vec2D secondDerivAt(double t) {
		return Vec2D.addVecs(p0.multiply(basisSecondDeriv0(t)), v0.multiply(basisSecondDeriv1(t)), a0.multiply(basisSecondDeriv2(t)),
				a1.multiply(basisSecondDeriv3(t)), v1.multiply(basisSecondDeriv4(t)), p1.multiply(basisSecondDeriv5(t)));
	}
}
