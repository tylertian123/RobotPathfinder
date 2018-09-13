package robot.pathfinder.follower;

/**
 * This is the base class for all the follower classes.
 * <p>
 * Followers are classes that can be given parameters to follow a specific trajectory.
 * They do so using a feedback control system, consisting of 4 gains: velocity feedforward,
 * acceleration feedforward, proportional gain, and derivative gain.
 * </p>
 * @author Tyler Tian
 *
 */
abstract public class Follower {
	
	protected double kA, kV, kP, kD;
	
	/**
	 * Sets the gains of the feedback control loop.
	 * @param kV The velocity feedforward
	 * @param kA The acceleration feedforward
	 * @param kP The proportional gain
	 * @param kD The derivative gain
	 */
	public void setGains(double kV, double kA, double kP, double kD) {
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kD = kD;
	}
	/**
	 * Sets the acceleration feedforward term of the control loop.
	 * @param a The acceleration feedforward
	 */
	public void setA(double a) {
		kA = a;
	}
	/**
	 * Sets the velocity feedforward term of the control loop.
	 * @param v The velocity feedforward
	 */
	public void setV(double v) {
		kV = v;
	}
	/**
	 * Sets the proportional gain of the control loop.
	 * @param p The proportional gain
	 */
	public void setP(double p) {
		kP = p;
	}
	/**
	 * Sets the derivative gain of the control loop.
	 * @param d The derivative gain
	 */
	public void setD(double d) {
		kD = d;
	}
	
	/**
	 * Initializes and starts the follower. This method must be called before {@link #run()} can be called.
	 */
	public abstract void initialize();
	/**
	 * Runs the control loop for one cycle. Note that this method must be called constantly so the control
	 * loop can run; generally, the more frequent the calls, the better the results. {@link #initialize()}
	 * must be called before this method can be called.
	 */
	public abstract void run();
	/**
	 * Stops the follower if it is already running. This will stop all motors.
	 */
	public abstract void stop();
	
	/**
	 * This functional interface is used by the follower to retrieve timestamps.
	 * @author Tyler Tian
	 *
	 */
	@FunctionalInterface
	public interface TimestampSource {
		public double getTimestamp();
	}
	@FunctionalInterface
	public interface DistanceSource {
		public double getDistance();
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
