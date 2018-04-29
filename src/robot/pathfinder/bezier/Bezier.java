package robot.pathfinder.bezier;

import robot.pathfinder.math.Vec2D;

public class Bezier {
	Vec2D[] controlPoints;
	
	public Bezier(Vec2D a, Vec2D b, Vec2D c, Vec2D d) {
		controlPoints = new Vec2D[4];
		controlPoints[0] = a;
		controlPoints[1] = b;
		controlPoints[2] = c;
		controlPoints[3] = d;
	}
	
	public static Bezier getFromHermite(Vec2D at0, Vec2D at1, Vec2D derivAt0, Vec2D derivAt1) {
		Vec2D p1 = at0.add(derivAt0.multiply(1.0 / 3.0));
		Vec2D p2 = at1.add(derivAt1.multiply(-1.0 / 3.0));
		return new Bezier(at0, p1, p2, at1);
	}
	
	public Vec2D at(double t) {
		double u = 1 - t;
		double uu = u * u;
		double uuu = u * u * u;
		double tt = t * t;
		double ttt = t * t * t;
		return Vec2D.addVecs(controlPoints[0].multiply(uuu), controlPoints[1].multiply(3 * uu * t),
				controlPoints[2].multiply(3 * u * tt), controlPoints[3].multiply(ttt));
	}
	public Vec2D derivAt(double t) {
		double u = 1 - t;
		double uu = u * u;
		double tt = t * t;
		return Vec2D.addVecs(controlPoints[1].subtract(controlPoints[0]).multiply(3 * uu),
				controlPoints[2].subtract(controlPoints[1]).multiply(6 * u * t), 
				controlPoints[3].subtract(controlPoints[2]).multiply(3 * tt));
	}
}
