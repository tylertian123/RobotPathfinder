package robot.pathfinder;

public class Moment {
	
	double d, v, a;
	double t;
	
	public Moment(double distance, double velocity, double acceleration) {
		d = distance;
		v = velocity;
		a = acceleration;
	}
	public Moment(double distance, double velocity, double acceleration, double t) {
		this(distance, velocity, acceleration);
		this.t = t;
	}
	
	public void setTime(double t) {
		this.t = t;
	}
	public double getTime() {
		return t;
	}
	public void setDistance(double distance) {
		d = distance;
	}
	public double getDistance() {
		return d;
	}
	public void setVelocity(double velocity) {
		v = velocity;
	}
	public double getVelocity() {
		return v;
	}
	public void setAcceleration(double acceleration) {
		a = acceleration;
	}
	public double getAcceleration() {
		return a;
	}
}
