package robot.pathfinder;

public class Moment {

	public double dist;
	public double velo;
	public double accel;
	public double t;
	
	public Moment() {
	}
	public Moment(double dist, double velo, double accel) {
		this(dist, velo, accel, 0);
	}
	public Moment(double dist, double velo, double accel, double t) {
		this.dist = dist;
		this.velo = velo;
		this.accel = accel;
		this.t = t;
	}
	
	public void setDist(double dist) {
		this.dist = dist;
	}
	public double getDist() {
		return dist;
	}
	public void setVelo(double velo) {
		this.velo = velo;
	}
	public double getVelo() {
		return velo;
	}
	public void setAccel(double accel) {
		this.accel = accel;
	}
	public double getAccel() {
		return accel;
	}
	public void setT(double t) {
		this.t = t;
	}
	public double getT() {
		return t;
	}
}
