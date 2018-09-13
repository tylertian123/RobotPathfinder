package robot.pathfinder.follower;

abstract public class Follower {
	
	protected double kA, kV, kP, kD;
	
	public void setGains(double kV, double kA, double kP, double kD) {
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kD = kD;
	}
	public void setA(double a) {
		kA = a;
	}
	public void setV(double v) {
		kV = v;
	}
	public void setP(double p) {
		kP = p;
	}
	public void setD(double d) {
		kD = d;
	}
	
	public abstract void initialize();
	public abstract void run();
	public abstract void stop();
	
	@FunctionalInterface
	public interface TimestampSource {
		public double getTimestamp();
	}
	public interface DistanceSource {
		public double getDistance();
		public void resetDistance();
	}
	@FunctionalInterface
	public interface DirectionSource {
		public double getDirection();
	}
	@FunctionalInterface
	public interface Motor {
		public void set(double speed);
	}
}
