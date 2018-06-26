package robot.pathfinder.math;

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
		// TODO Auto-generated method stub
		return null;
	}
	@Override
	public Vec2D secondDerivAt(double t) {
		// TODO Auto-generated method stub
		return null;
	}
}
