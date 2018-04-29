package robot.pathfinder.math;

public class Circle {
	
	Vec2D center;
	double radius;
	
	public Circle(Vec2D center, double radius) {
		this.center = center;
		this.radius = radius;
	}
	
	public Vec2D getCenter() {
		return center;
	}
	public double getRadius() {
		return radius;
	}
}
