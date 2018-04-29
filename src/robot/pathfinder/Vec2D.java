package robot.pathfinder;

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
	
	public Vec2D multiplyBy(double scalar) {
		return new Vec2D(x * scalar, y * scalar);
	}
	public Vec2D addTo(Vec2D vec) {
		return new Vec2D(x + vec.x, y + vec.y);
	}
	
}
