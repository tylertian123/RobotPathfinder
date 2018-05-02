package robot.pathfinder;

public class TankDrivePathSegment {
	
	double start, end;
	double leftMaxVelocity, rightMaxVelocity;
	
	public TankDrivePathSegment(double start, double end) {
		this.start = start;
		this.end = end;
	}
	public TankDrivePathSegment(double start, double end, double leftMaxVelocity, double rightMaxVelocity) {
		this(start, end);
		this.leftMaxVelocity = leftMaxVelocity;
		this.rightMaxVelocity = rightMaxVelocity;
	}
	
	public void setMaxVelocities(double leftMaxVelocity, double rightMaxVelocity) {
		this.leftMaxVelocity = leftMaxVelocity;
		this.rightMaxVelocity = rightMaxVelocity;
	}
	public double getLeftMaxVelocity() {
		return leftMaxVelocity;
	}
	public double getRightMaxVelocity() {
		return rightMaxVelocity;
	}
	public double getStart() {
		return start;
	}
	public double getEnd() {
		return end;
	}
}
