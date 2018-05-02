package robot.pathfinder.math;

public class Vec2D {
	
	double x, y;
	
	public Vec2D(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double getX() {
		return x;
	}
	public double getY() {
		return y;
	}
	
	public static Vec2D addVecs(Vec2D... vecs) {
		Vec2D v = new Vec2D(0, 0);
		for(Vec2D vec : vecs) {
			v.x += vec.x;
			v.y += vec.y;
		}
		return v;
	}
	
	public static double dist(Vec2D a, Vec2D b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}
	
	public Vec2D multiply(double scalar) {
		return new Vec2D(x * scalar, y * scalar);
	}
	public Vec2D add(Vec2D vec) {
		return new Vec2D(x + vec.x, y + vec.y);
	}
	public Vec2D subtract(Vec2D vec) {
		return new Vec2D(x - vec.x, y - vec.y);
	}
	public double distTo(Vec2D vec) {
		return Vec2D.dist(this, vec);
	}
}
