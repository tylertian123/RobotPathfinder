package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

/**
 * This is the base class for all the follower classes.
 * <p>
 * A follower is a special class that can be set to follow a {@link Followable}
 * object (that is, actually driving out the trajectory with a robot in real
 * life).
 * </p>
 * <p>
 * They do so by using both feedback and feedforward control, with a system that
 * consists of 5 parts: proportional, integral and derivative gains on the
 * position error, and velocity and acceleration feedforward (sometimes known as
 * PIDVA control). For more information regarding the gains, see the
 * documentation for the fields of {@link Gains}.
 * </p>
 * <p>
 * To use a follower, one first must provide values for the aforementioned
 * gains, sensors for the feedback loop such as encoders, and a timer, such as
 * {@link System#currentTimeMillis()}. Then, call {@link #initialize()} to
 * initialize the follower. After initialization, repeatedly call {@link #run()}
 * to run the follower. Higher frequencies should lead to better results, but
 * only up to a certain limit. When {@link #isFinished()} returns true, the
 * following is completed. To stop the follower before it normally finishes,
 * call {@link #stop()}.
 * </p>
 * 
 * @param <T> The type of moment used by this {@link Follower}; must be a
 *            subclass of {@link Moment}
 * @author Tyler Tian
 * @since 3.0.0
 * @see Followable
 */
abstract public class Follower<T extends Moment> {

	protected double kA, kV, kP, kI, kD;
	protected Followable<T> target;
	protected TimestampSource timer;

	protected boolean running = false;
	protected boolean finished = false;

	/**
	 * A class that represents a set of gains for PIDVA control.
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 */
	public static class Gains implements Cloneable {

		/**
		 * The velocity feedforward value.
		 * <p>
		 * This value is multiplied by the desired (setpoint) velocity, and then added
		 * to the output at each point in time.
		 * </p>
		 * <p>
		 * The default value for this term is 0.
		 * </p>
		 */
		public double kV = 0;

		/**
		 * The acceleration feedforward value.
		 * <p>
		 * This value is multiplied by the desired (setpoint) acceleration, and then
		 * added to the output at each point in time.
		 * </p>
		 * <p>
		 * The default value for this term is 0.
		 * </p>
		 */
		public double kA = 0;

		/**
		 * The proportional feedback value.
		 * <p>
		 * This value is multiplied by the error in position (the difference between the
		 * setpoint's position and the current position), and then added to the output
		 * at each point in time.
		 * </p>
		 * <p>
		 * The default value for this term is 0.
		 * </p>
		 */
		public double kP = 0;

		/**
		 * The integral feedback value.
		 * <p>
		 * This value is multiplied by the integral of the error (the error accumulated
		 * through time), and then added to the output at each point in time.
		 * </p>
		 * <p>
		 * The default value for this term is 0.
		 * </p>
		 */
		public double kI = 0;

		/**
		 * The derivative feedback value.
		 * <p>
		 * This value is multiplied by the derivative of the error (the rate of change
		 * of the error), and then added to the output at each point in time.
		 * </p>
		 * <p>
		 * The default value for this term is 0.
		 * </p>
		 */
		public double kD = 0;

		/**
		 * Creates an identical copy of this {@link Gains} object.
		 */
		@Override
		public Gains clone() {
			Gains gains = new Gains();
			gains.kV = kV;
			gains.kA = kA;
			gains.kP = kP;
			gains.kI = kI;
			gains.kD = kD;

			return gains;
		}

		/**
		 * Constructs a new set of gains with each gain set to 0.
		 */
		public Gains() {
		}

		/**
		 * Constructs a new set of gains with each gain set to the specified value.
		 * 
		 * @param kV The {@link #kV velocity feedforward}
		 * @param kA The {@link #kA acceleration feedforward}
		 * @param kP The {@link #kP proportional feedback}
		 * @param kI The {@link #kI integral feedback}
		 * @param kD The {@link #kD derivative feedback}
		 */
		public Gains(double kV, double kA, double kP, double kI, double kD) {
			this.kV = kV;
			this.kA = kA;
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
		}
	}

	/**
	 * This method must be implemented by any concrete follower to perform any
	 * initialization tasks. (e.g. resetting time references)
	 */
	protected abstract void _initialize();

	/**
	 * This method must be implemented by any concrete follower to run one iteration
	 * of the control loop. This method should return whether or not the follower is
	 * finished.
	 * 
	 * @return Whether the follower has finished running
	 */
	protected abstract boolean _run();

	/**
	 * This method must be implemented by any concrete follower to perform any
	 * additional cleanup after the follower has stopped. (e.g. stopping all motors)
	 */
	protected abstract void _stop();

	/**
	 * Gets whether the follower is running. The follower is considered to be
	 * "running" if {@link #initialize()} has been called, the trajectory did not
	 * end, <em>and</em> {@link #stop()} has not been called.
	 * <p>
	 * This method behaves similarly to {@link #isFinished()}. Before
	 * {@link #initialize()} is called for the first time, both {@link #isRunning()}
	 * and {@link #isFinished()} return false. Once {@link #initialize()} is called,
	 * this method will always return the opposite of {@link #isFinished()}.
	 * </p>
	 * 
	 * @return Whether the follower is running
	 * @see #isFinished()
	 */
	public boolean isRunning() {
		return running;
	}

	/**
	 * Gets whether the follower has finished. The follower is considered to be
	 * "finished" if {@link #stop()} has been called, <em>or</em> the trajectory has
	 * ended. Additionally, the finished state is reset when {@link #initialize()}
	 * is called.
	 * <p>
	 * This method behaves similarly to {@link #isRunning()}. Before
	 * {@link #initialize()} is called for the first time, both
	 * {@link #isFinished()} and {@link #isRunning()} return false. Once
	 * {@link #initialize()} is called, this method will always return the opposite
	 * of {@link #isRunning()}.
	 * </p>
	 * 
	 * @return Whether the follower has finished
	 * @see #isRunning()
	 */
	public boolean isFinished() {
		return finished;
	}

	/**
	 * Initializes and starts the follower. This method should be called before
	 * {@link #run()} can be called.<br>
	 * <br>
	 * If the follower is currently running, this method will do nothing.
	 */
	public void initialize() {
		if (running) {
			return;
		}
		_initialize();
		running = true;
		finished = false;
	}

	/**
	 * Runs the control loop for one cycle. Note that this method must be called
	 * constantly so the control loop can run; generally, the more frequent the
	 * calls, the better the results. {@link #initialize()} should be called before
	 * this method can be called.<br>
	 * <br>
	 * If the follower is not initialized (not running), this method will first call
	 * {@link #initialize()} and then perform one cycle of the control loop.
	 * 
	 * @see #initialize()
	 */
	public void run() {
		if (!running) {
			// If already finished, do nothing
			if (finished) {
				return;
			}
			// Otherwise initialize
			initialize();
		}

		if (_run()) {
			stop();
		}
	}

	/**
	 * Stops the follower if it is already running.
	 */
	public void stop() {
		_stop();
		running = false;
		finished = true;
	}

	/**
	 * Sets the target to follow.
	 * 
	 * @param target The new target to follow
	 * @throws IllegalStateException If the follower is running
	 */
	public void setTarget(Followable<T> target) {
		if (running) {
			throw new IllegalStateException("Target cannot be changed when follower is running");
		}
		this.target = target;
	}

	/**
	 * Gets the target to follow.
	 * 
	 * @return The target followed by this follower
	 */
	public Followable<T> getTarget() {
		return target;
	}

	/**
	 * Sets the gains of the feedback control loop.
	 * 
	 * @param gains A {@link Gains} object containing the gains to set.
	 */
	public void setGains(Gains gains) {
		this.kV = gains.kV;
		this.kA = gains.kA;
		this.kP = gains.kP;
		this.kI = gains.kI;
		this.kD = gains.kD;
	}

	/**
	 * Gets the gains of the feedback control loop.
	 * 
	 * @return A {@link Gains} object containing the gains of the system.
	 */
	public Gains getGains() {
		return new Gains(kV, kA, kP, kI, kD);
	}

	/**
	 * Sets the gains of the feedback control loop.
	 * 
	 * @param kV The velocity feedforward coefficient
	 * @param kA The acceleration feedforward coefficient
	 * @param kP The proportional gain
	 * @param kI the integral gain
	 * @param kD The derivative gain
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setGains(double kV, double kA, double kP, double kI, double kD) {
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	/**
	 * Sets the acceleration feedforward term's coefficient of the control loop.
	 * 
	 * @param a The acceleration feedforward term's coefficient
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setA(double a) {
		kA = a;
	}

	/**
	 * Retrieves the acceleration feedforward term's coefficient of the control
	 * loop.
	 * 
	 * @return The acceleration feedforward term coefficient
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getA() {
		return kA;
	}

	/**
	 * Sets the velocity feedforward term's coefficient of the control loop.
	 * 
	 * @param v The velocity feedforward term's coefficient
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setV(double v) {
		kV = v;
	}

	/**
	 * Retrieves the velocity feedforward term's coefficient of the control loop.
	 * 
	 * @return The velocity feedforward term coefficient
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getV() {
		return kV;
	}

	/**
	 * Sets the proportional gain of the control loop.
	 * 
	 * @param p The proportional gain
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setP(double p) {
		kP = p;
	}

	/**
	 * Retrieves the proportional gain of the control loop.
	 * 
	 * @return The proportional gain
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getP() {
		return kP;
	}

	/**
	 * Sets the integral gain of the control loop.
	 * 
	 * @param i The integral gain
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setI(double i) {
		kI = i;
	}

	/**
	 * Retrieves the integral gain of the control loop.
	 * 
	 * @return The integral gain
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getI() {
		return kI;
	}

	/**
	 * Sets the derivative gain of the control loop.
	 * 
	 * @param d The derivative gain
	 * @deprecated Use {@link #setGains(Gains)} instead.
	 */
	@Deprecated
	public void setD(double d) {
		kD = d;
	}

	/**
	 * Retrieves the derivative gain of the control loop.
	 * 
	 * @return The derivative gain
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getD() {
		return kD;
	}

	/**
	 * This functional interface represents a source of timestamp data, such as a
	 * FPGA timer.
	 * <p>
	 * The timestamps are used to calculate the derivative part of the control loop;
	 * therefore, it is recommended that they have a resolution of at least one
	 * millisecond (preferably higher). Higher resolutions will yield better
	 * results.
	 * </p>
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 */
	@FunctionalInterface
	public interface TimestampSource {
		/**
		 * Gets the current time.
		 * <p>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should be in
		 * seconds.
		 * </p>
		 * 
		 * @return The current time
		 */
		public double getTimestamp();
	}

	/**
	 * This functional interface represents a source of 1-dimensional position data,
	 * such as an encoder.
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 */
	@FunctionalInterface
	public interface PositionSource {
		/**
		 * Gets position data from the source.
		 * <p>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should be in
		 * meters.
		 * </p>
		 * 
		 * @return The current position
		 */
		public double getPosition();
	}

	/**
	 * This interface is an extension of {@link PositionSource}. It represents a
	 * source of 1-dimensional position, velocity and acceleration data.
	 * <p>
	 * This class is used by implementations of {@link DynamicFollower}.
	 * </p>
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 * @see PositionSource
	 * @see DynamicFollower
	 */
	public interface AdvancedPositionSource extends PositionSource {
		/**
		 * Gets velocity data from the source.
		 * <p>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should also be in
		 * m/s.
		 * </p>
		 * 
		 * @return The current velocity
		 */
		public double getVelocity();

		/**
		 * Gets acceleration data from the source.
		 * <p>
		 * Please note that the unit of the result should stay consistent with the unit
		 * used to generate the trajectory the follower is to follow. For example, if
		 * the trajectory is generated with units of m/s, the result should also be in
		 * m/s^2.
		 * </p>
		 * 
		 * @return The current acceleration
		 */
		public double getAcceleration();
	}

	/**
	 * This functional interface represents a source of orientation/directional
	 * data, such as a gyroscope.
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 */
	@FunctionalInterface
	public interface DirectionSource {
		/**
		 * Gets orientation/directional data from the source.
		 * <p>
		 * Please note that the result should be in radians, with 0 representing right.
		 * The representation is the same as the angles used to generate the trajectory.
		 * </p>
		 * 
		 * @return The angle the robot is facing
		 */
		public double getDirection();
	}

	/**
	 * This functional interface represents a motor or any kind of device that will
	 * accept the output of the follower.
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 */
	@FunctionalInterface
	public interface Motor {
		/**
		 * Sets the output of the motor.
		 * <p>
		 * The output should be a number between -1 and 1, with -1 being full speed
		 * reverse, 1 being full speed forward, and 0 being no motion.
		 * </p>
		 * 
		 * @param output The output to set the motor to
		 */
		public void set(double output);
	}
}
