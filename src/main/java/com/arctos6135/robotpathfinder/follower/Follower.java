package com.arctos6135.robotpathfinder.follower;

/**
 * This is the base class for all the follower classes.
 * <p>
 * Followers are classes that can be given parameters to follow a specific
 * trajectory. They do so using a feedback control system, consisting of 4
 * gains: velocity feedforward, acceleration feedforward, proportional gain, and
 * derivative gain.
 * </p>
 * 
 * @author Tyler Tian
 *
 */
abstract public class Follower {

	protected double kA, kV, kP, kD;

	/**
	 * Sets the gains of the feedback control loop.
	 * 
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
	 * 
	 * @param a The acceleration feedforward
	 */
	public void setA(double a) {
		kA = a;
	}

	/**
	 * Sets the velocity feedforward term of the control loop.
	 * 
	 * @param v The velocity feedforward
	 */
	public void setV(double v) {
		kV = v;
	}

	/**
	 * Sets the proportional gain of the control loop.
	 * 
	 * @param p The proportional gain
	 */
	public void setP(double p) {
		kP = p;
	}

	/**
	 * Sets the derivative gain of the control loop.
	 * 
	 * @param d The derivative gain
	 */
	public void setD(double d) {
		kD = d;
	}

	/**
	 * Tests whether the follower is running.
	 * 
	 * @return Whether the follower is running
	 */
	public abstract boolean isRunning();

	/**
	 * Initializes and starts the follower. This method must be called before
	 * {@link #run()} can be called.
	 */
	public abstract void initialize();

	/**
	 * Runs the control loop for one cycle. Note that this method must be called
	 * constantly so the control loop can run; generally, the more frequent the
	 * calls, the better the results. {@link #initialize()} must be called before
	 * this method can be called.
	 */
	public abstract void run();

	/**
	 * Stops the follower if it is already running. This will stop all motors.
	 */
	public abstract void stop();

	/**
	 * This functional interface represents a source of timestamp data, such as a
	 * FPGA timer.<br>
	 * The timestamps are used to calculate the derivative part of the control loop;
	 * therefore, it is recommended that they have a resolution of at least one
	 * millisecond (preferably higher). Higher resolutions will yield better
	 * results.
	 * 
	 * @author Tyler Tian
	 *
	 */
	@FunctionalInterface
	public interface TimestampSource {
		/**
		 * Gets a timestamp from the source.<br>
		 * <br>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should be in
		 * seconds.
		 * 
		 * @return The timestamp
		 */
		public double getTimestamp();
	}

	/**
	 * This functional interface represents a source of distance data, such as an
	 * encoder.
	 * 
	 * @author Tyler Tian
	 *
	 */
	@FunctionalInterface
	public interface DistanceSource {
		/**
		 * Gets distance data from the source.<br>
		 * <br>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should be in
		 * meters.
		 * 
		 * @return The distance
		 */
		public double getDistance();
	}

	/**
	 * This functional interface represents a source of orientation/directional
	 * data, such as a gyroscope.
	 * 
	 * @author Tyler Tian
	 *
	 */
	@FunctionalInterface
	public interface DirectionSource {
		/**
		 * Gets orientation/directional data from the source.<br>
		 * <br>
		 * Please note that the result should be in radians, with 0 representing right.
		 * The representation should be the same as the angles used to generate the
		 * trajectory.
		 * 
		 * @return The angle the robot is facing
		 */
		public double getDirection();
	}

	/**
	 * This functional interface represents a motor or any kind of device that will
	 * accept the output.
	 * 
	 * @author Tyler Tian
	 *
	 */
	@FunctionalInterface
	public interface Motor {
		/**
		 * Sets the speed of the motor. The speed should be a number between -1 and 1,
		 * with -1 being full speed reverse, 1 being full speed forward, and 0 being no
		 * motion.
		 * 
		 * @param speed The speed to set the motor to
		 */
		public void set(double speed);
	}
}
