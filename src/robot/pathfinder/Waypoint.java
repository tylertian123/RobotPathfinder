package robot.pathfinder;

import robot.pathfinder.math.Vec2D;

public class Waypoint {
	double x, y, heading;
	
	public Waypoint(double x, double y) {
		this.x = x;
		this.y = y;
	}
	public Waypoint(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
	
	public double getX() {
		return x;
	}
	public double getY() {
		return y;
	}
	public double getHeading() {
		return heading;
	}
	public Vec2D getPosVec() {
		return new Vec2D(x, y);
	}
}
