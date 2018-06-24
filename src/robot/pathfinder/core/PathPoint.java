package robot.pathfinder.core;

public class PathPoint {
	
	double location;
	double maxVel;
	
	public PathPoint(double location, double maxVel) {
		this.location = location;
		this.maxVel = maxVel;
	}
	public PathPoint(double location) {
		this(location, 0);
	}
	public PathPoint() {
		this(0, 0);
	}
	
	public double getLocation() {
		return location;
	}
	public void setLocation(double location) {
		this.location = location;
	}
	public double getMaxVel() {
		return maxVel;
	}
	public void setMaxVel(double maxVel) {
		this.maxVel = maxVel;
	}
}
