package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.math.MathUtils;

/**
 * A follower class for tank drive robots.
 * <p>
 * See {@link Follower} for the basic definition of a "follower". This class
 * provides a concrete implementation of the abstract class {@link Follower} for
 * {@link TankDriveMoment}.
 * </p>
 * <p>
 * In addition to the 5 terms mentioned in the documentation for
 * {@link Follower}, {@link TankDriveFollower}s also have a fifth term&mdash;the
 * "directional-proportional" (DP) term. This term takes the error between the
 * angle the robot is supposed to be facing and the angle it's actually facing,
 * multiplies the error by a constant, and subtracts it from the left wheel's
 * output while adding it to the right wheel's output in order to correct the
 * overall direction the robot is facing.
 * </p>
 * <p>
 * See the documentation for {@link Follower} for usage instructions.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 * @see Follower
 */
public class TankDriveFollower extends Follower<TankDriveMoment> {

	protected PositionSource lDistSrc, rDistSrc;
	protected DirectionSource directionSrc;
	protected Motor lMotor, rMotor;

	// Directional proportional gain
	protected double kDP = 0;

	// Keep track of the initial timestamp and distance measurements so we don't
	// have to reset
	// Keep track of the error and timestamp of the last iteration to calculate the
	// derivative
	// Keep track of the accumulated error integral
	protected double initTime, lastTime, lLastErr, rLastErr, lInitDist, rInitDist, initDirection, lErrorInt, rErrorInt;

	protected double leftErr, rightErr, dirErr;
	// Store these as member variables so they can be accessed from outside the
	// class for testing purposes
	protected double leftOutput, rightOutput, leftDeriv, rightDeriv;
	protected TankDriveMoment lastMoment;

	/**
	 * A class that represents a set of gains for PIDVA control, specialized for
	 * tank drive robots.
	 * 
	 * In addition to the 5 terms in {@link Gains}, this class also includes a fifth
	 * term, the directional-proportional term. For more information, see the
	 * documentation for {@link #kDP the field}.
	 * 
	 * @author Tyler Tian
	 * @since 3.0.0
	 * @see Gains
	 */
	public static class TankDriveGains extends Gains {

		/**
		 * The directional-proportional term.
		 * 
		 * This term is multiplied by the error between the angle the robot is supposed
		 * to be facing and the angle it's actually facing, and is subtracted from the
		 * left wheel's output and added to the right wheel's output in order to correct
		 * the overall direction the robot is facing.
		 * 
		 * The default value for this term is 0.
		 */
		public double kDP = 0;

		/**
		 * {@inheritDoc}
		 */
		@Override
		public TankDriveGains clone() {
			TankDriveGains gains = new TankDriveGains();
			gains.kV = kV;
			gains.kA = kA;
			gains.kP = kP;
			gains.kI = kI;
			gains.kD = kD;
			gains.kDP = kDP;

			return gains;
		}

		/**
		 * Constructs a new set of gains with each gain set to 0.
		 */
		public TankDriveGains() {
		}

		/**
		 * Constructs a new set of gains with each gain set to the specified value.
		 * 
		 * @param kV  The {@link #kV velocity feedforward}
		 * @param kA  The {@link #kA acceleration feedforward}
		 * @param kP  The {@link #kP proportional feedback}
		 * @param kI  The {@link #kI integral feedback}
		 * @param kD  The {@link #kD derivative feedback}
		 * @param kDP The {@link #kDP directional-proportional feedback}
		 */
		public TankDriveGains(double kV, double kA, double kP, double kI, double kD, double kDP) {
			this.kV = kV;
			this.kA = kA;
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.kDP = kDP;
		}
	}

	public static class TankDriveRobot implements Cloneable {

		public Motor leftMotor, rightMotor;

		public PositionSource leftPositionSource, rightPositionSource;

		public TimestampSource timestampSource;

		public DirectionSource directionSource;

		@Override
		public TankDriveRobot clone() {
			return new TankDriveRobot(leftMotor, rightMotor, leftPositionSource, rightPositionSource, timestampSource,
					directionSource);
		}

		public TankDriveRobot() {
		}

		public TankDriveRobot(Motor lMotor, Motor rMotor, PositionSource lPosSrc, PositionSource rPosSrc,
				TimestampSource timeSrc, DirectionSource dirSrc) {
			leftMotor = lMotor;
			rightMotor = rMotor;
			leftPositionSource = lPosSrc;
			rightPositionSource = rPosSrc;
			timestampSource = timeSrc;
			directionSource = dirSrc;
		}
	}

	/**
	 * Constructs a new tank drive follower only using the feedforward terms (VA).
	 * <p>
	 * This constructor does not require any sensors for distance/position or
	 * direction, or any values for kP, kI, kD, and kDP. Therefore, all of the
	 * following is based on feedforward terms. Be warned that while this may be
	 * quite inaccurate as it has no sensor feedback.
	 * <p>
	 * 
	 * @param target The target to follow
	 * @param lMotor The left side motor
	 * @param rMotor The right side motor
	 * @param timer  A {@link TimestampSource} to get the current time from
	 * @param kV     The velocity feedforward
	 * @param kA     The acceleration feedforward
	 * @deprecated Use
	 *             {@link #TankDriveFollower(Followable, Motor, Motor, TimestampSource, Gains)}
	 *             instead.
	 */
	@Deprecated
	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, TimestampSource timer,
			double kV, double kA) {
		setGains(kV, kA, 0, 0, 0, 0);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = null;
		this.rDistSrc = null;
		this.timer = timer;
		this.directionSrc = null;
	}

	/**
	 * Constructs a new tank drive follower using PIDVA terms, but not the DP term.
	 * <p>
	 * This constructor does not require any sensors for direction, or a value for
	 * kDP. Therefore, the directional-proportional term is not used. This may
	 * reduce accuracy, but is typically good enough.
	 * </p>
	 * 
	 * @param target   The target to follow
	 * @param lMotor   The left side motor
	 * @param rMotor   The right side motor
	 * @param lDistSrc A {@link PositionSource} for the left motor
	 * @param rDistSrc A {@link PositionSource} for the right motor
	 * @param timer    A {@link TimestampSource} to get the current time from
	 * @param kV       The velocity feedforward
	 * @param kA       The acceleration feedforward
	 * @param kP       The proportional gain
	 * @param kI       The integral gain
	 * @param kD       The derivative gain
	 * @deprecated Use
	 *             {@link #TankDriveFollower(Followable, Motor, Motor, PositionSource, PositionSource, TimestampSource, Gains)}
	 *             instead.
	 */
	@Deprecated
	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, PositionSource lDistSrc,
			PositionSource rDistSrc, TimestampSource timer, double kV, double kA, double kP, double kI, double kD) {
		setGains(kV, kA, kP, kI, kD, 0);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = null;
	}

	/**
	 * Constructs a new tank drive follower using the feedforward terms and the
	 * directional-proportional term (VA + DP).
	 * <p>
	 * This constructor does not require any sensors for distance/position, or
	 * values for kP, kI and kD. Therefore, all of the following is based on the
	 * feedforward terms and the directional-proportional term. This may be
	 * inaccurate.
	 * </p>
	 * 
	 * @param target The target to follow
	 * @param lMotor The left side motor
	 * @param rMotor The right side motor
	 * @param timer  A {@link TimestampSource} to get the current time from
	 * @param dirSrc A {@link DirectionSource} to get angle data from
	 * @param kV     The velocity feedforward
	 * @param kA     The acceleration feedforward
	 * @param kDP    The directional-proportional gain
	 * @deprecated Use
	 *             {@link #TankDriveFollower(Followable, Motor, Motor, TimestampSource, DirectionSource, Gains)}
	 *             instead.
	 */
	@Deprecated
	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, TimestampSource timer,
			DirectionSource dirSrc, double kV, double kA, double kDP) {
		setGains(kV, kA, 0, 0, 0, kDP);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = null;
		this.rDistSrc = null;
		this.timer = timer;
		this.directionSrc = dirSrc;
	}

	/**
	 * Constructs a new tank drive follower using all the terms.
	 * <p>
	 * Since this constructor uses all the terms (PIDVA + DP), it is the most
	 * accurate.
	 * </p>
	 * 
	 * @param target   The target to follow
	 * @param lMotor   The left side motor
	 * @param rMotor   The right side motor
	 * @param lDistSrc A {@link PositionSource} for the left motor
	 * @param rDistSrc A {@link PositionSource} for the right motor
	 * @param timer    A {@link TimestampSource} to get the current time from
	 * @param dirSrc   A {@link DirectionSource} to get angle data from
	 * @param kV       The velocity feedforward
	 * @param kA       The acceleration feedforward
	 * @param kP       The proportional gain
	 * @param kI       The integral gain
	 * @param kD       The derivative gain
	 * @param kDP      The directional-proportional gain
	 * @deprecated Use
	 *             {@link #TankDriveFollower(Followable, Motor, Motor, PositionSource, PositionSource, TimestampSource, DirectionSource, Gains)}
	 *             instead.
	 */
	@Deprecated
	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, PositionSource lDistSrc,
			PositionSource rDistSrc, TimestampSource timer, DirectionSource dirSrc, double kV, double kA, double kP,
			double kI, double kD, double kDP) {
		setGains(kV, kA, kP, kI, kD, kDP);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = dirSrc;
	}

	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, TimestampSource timer,
			Gains gains) {
		setGains(gains);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.timer = timer;
	}

	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, PositionSource lDistSrc,
			PositionSource rDistSrc, TimestampSource timer, Gains gains) {
		setGains(gains);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
	}

	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, TimestampSource timer,
			DirectionSource dirSrc, Gains gains) {
		setGains(gains);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.timer = timer;
		this.directionSrc = dirSrc;
	}

	public TankDriveFollower(Followable<TankDriveMoment> target, Motor lMotor, Motor rMotor, PositionSource lDistSrc,
			PositionSource rDistSrc, TimestampSource timer, DirectionSource dirSrc, Gains gains) {
		setGains(gains);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = dirSrc;
	}

	/**
	 * {@inheritDoc}
	 * 
	 * Note that if the object passed in is an instance of {@link TankDriveGains},
	 * this method will call {@link #setGains(TankDriveGains)}.
	 */
	@Override
	public void setGains(Gains gains) {
		if (gains instanceof TankDriveGains) {
			setGains((TankDriveGains) gains);
		} else {
			super.setGains(gains);
		}
	}

	/**
	 * Sets the gains of the feedback control loop.
	 * 
	 * @param gains A {@link TankDriveGains} object containing the gains to set.
	 */
	public void setGains(TankDriveGains gains) {
		super.setGains((Gains) gains);
		kDP = gains.kDP;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public TankDriveGains getGains() {
		return new TankDriveGains(kV, kA, kP, kI, kD, kDP);
	}

	/**
	 * Sets the gains of the feedback loop.
	 * 
	 * @param kV  The velocity feedforward
	 * @param kA  The acceleration feedforward
	 * @param kP  The proportional gain
	 * @param kI  The integral gain
	 * @param kD  The derivative gain
	 * @param kDP The directional-proportional gain
	 * @deprecated Use {@link #setGains(TankDriveGains)} instead.
	 */
	@Deprecated
	public void setGains(double kV, double kA, double kP, double kI, double kD, double kDP) {
		setGains(kV, kA, kP, kI, kD);
		setDP(kDP);
	}

	/**
	 * Sets the directional-proportional gain of the feedback loop.
	 * <p>
	 * This term takes the error between the angle the robot is supposed to be
	 * facing and the angle it's actually facing, multiplies the error by a
	 * constant, and subtracts it from the left wheels output while adding it to the
	 * right wheel's output in order to correct the overall direction the robot is
	 * facing.
	 * </p>
	 * 
	 * @param kDP The directional-proportional gain
	 * @deprecated Use {@link #setGains(TankDriveGains)} instead.
	 */
	@Deprecated
	public void setDP(double kDP) {
		this.kDP = kDP;
	}

	/**
	 * Gets the directional-proportional gain of the feedback loop.
	 * <p>
	 * This term takes the error between the angle the robot is supposed to be
	 * facing and the angle it's actually facing, multiplies the error by a
	 * constant, and subtracts it from the left wheels output while adding it to the
	 * right wheel's output in order to correct the overall direction the robot is
	 * facing.
	 * </p>
	 * 
	 * @return The directional-proportional gain
	 * @deprecated Use {@link #getGains()} instead.
	 */
	@Deprecated
	public double getDP() {
		return kDP;
	}

	/**
	 * Sets the timestamp source.
	 * 
	 * @param timer The timestamp source
	 * @throws IllegalStateException If the follower is running
	 */
	public void setTimestampSource(TimestampSource timer) {
		if (running) {
			throw new IllegalStateException("Timestamp Source cannot be changed when follower is running");
		}
		this.timer = timer;
	}

	/**
	 * Sets the motors.
	 * 
	 * @param lMotor The left motor
	 * @param rMotor The right motor
	 * @throws IllegalStateException If the follower is running
	 */
	public void setMotors(Motor lMotor, Motor rMotor) {
		if (running) {
			throw new IllegalStateException("Motors cannot be changed when follower is running");
		}
		this.lMotor = lMotor;
		this.rMotor = rMotor;
	}

	/**
	 * Sets the position sources.
	 * 
	 * @param lDistSrc The left position source
	 * @param rDistSrc The right position source
	 * @throws IllegalStateException If the follower is running
	 */
	public void setDistanceSources(PositionSource lDistSrc, PositionSource rDistSrc) {
		if (running) {
			throw new IllegalStateException("Position Sources cannot be changed when follower is running");
		}
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
	}

	@Override
	protected void _initialize() {
		// Reset the initial distance, direction and timestamp references
		if (lDistSrc != null && rDistSrc != null) {
			lInitDist = lDistSrc.getPosition();
			rInitDist = rDistSrc.getPosition();
		}
		if (directionSrc != null) {
			initDirection = directionSrc.getDirection();
		}
		initTime = lastTime = timer.getTimestamp();

		// Reset integrals and last errors
		lErrorInt = rErrorInt = lLastErr = rLastErr = 0;
	}

	@Override
	protected boolean _run() {
		// Calculate current t and time difference from last iteration
		double timestamp = timer.getTimestamp();
		double dt = timestamp - lastTime;
		double t = timestamp - initTime;
		if (t > target.totalTime()) {
			return true;
		}

		TankDriveMoment m = target.get(t);

		leftErr = rightErr = leftDeriv = rightDeriv = dirErr = 0;
		// Calculate errors and derivatives only if the distance sources are not null
		if (lDistSrc != null && rDistSrc != null) {
			// Calculate left and right errors
			leftErr = m.getLeftPosition() - (lDistSrc.getPosition() - lInitDist);
			rightErr = m.getRightPosition() - (rDistSrc.getPosition() - rInitDist);
			// Get the derivative of the errors
			leftDeriv = (leftErr - lLastErr) / dt;
			rightDeriv = (rightErr - rLastErr) / dt;
			// Calculate the integral of the error
			lErrorInt += leftErr * dt;
			rErrorInt += rightErr * dt;
		}
		// Calculate directional error only if the direction source is not null
		if (directionSrc != null) {
			// This angle diff will be positive if the robot needs to turn left
			dirErr = MathUtils.angleDiff(directionSrc.getDirection() - initDirection, m.getFacingRelative());
		}
		// Calculate outputs
		leftOutput = kA * m.getLeftAcceleration() + kV * m.getLeftVelocity() + kP * leftErr + kI * lErrorInt
				+ kD * leftDeriv - dirErr * kDP;
		rightOutput = kA * m.getRightAcceleration() + kV * m.getRightVelocity() + kP * rightErr + kI * lErrorInt
				+ kD * rightDeriv + dirErr * kDP;
		// Constrain
		leftOutput = Math.max(-1, Math.min(1, leftOutput));
		rightOutput = Math.max(-1, Math.min(1, rightOutput));

		lMotor.set(leftOutput);
		rMotor.set(rightOutput);

		lastTime = t;
		lLastErr = leftErr;
		rLastErr = rightErr;

		lastMoment = m;

		return false;
	}

	@Override
	protected void _stop() {
		lMotor.set(0);
		rMotor.set(0);
	}

	/**
	 * Retrieves the last positional error of the left wheel.
	 * <p>
	 * This value is multiplied by <b>kP</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last left positional error
	 */
	public double lastLeftError() {
		return leftErr;
	}

	/**
	 * Retrieves the last positional error of the right wheel.
	 * <p>
	 * This value is multiplied by <b>kP</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last right positional error
	 */
	public double lastRightError() {
		return rightErr;
	}

	/**
	 * Retrieves the last calculated integral of the error of the left wheel.
	 * <p>
	 * This value is multiplied by <b>kI</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last left error integral
	 */
	public double lastLeftIntegral() {
		return lErrorInt;
	}

	/**
	 * Retrieves the last calculated integral of the error of the right wheel.
	 * <p>
	 * This value is multiplied by <b>kI</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last right error integral
	 */
	public double lastRightIntegral() {
		return rErrorInt;
	}

	/**
	 * Retrieves the last directional error of the robot.
	 * <p>
	 * This value is multiplied by <b>kDP</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last directional error
	 */
	public double lastDirectionalError() {
		return dirErr;
	}

	/**
	 * Retrieves the last derivative error of the left wheel.
	 * <p>
	 * This value is multiplied by <b>kD</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last left derivative error
	 */
	public double lastLeftDerivative() {
		return leftDeriv;
	}

	/**
	 * Retrieves the last derivative error of the right wheel.
	 * <p>
	 * This value is multiplied by <b>kD</b>, and added to the left output.
	 * </p>
	 * 
	 * @return The last right derivative error
	 */
	public double lastRightDerivative() {
		return rightDeriv;
	}

	/**
	 * Retrieves the last output written to the left motor.
	 * 
	 * @return The last left output
	 */
	public double lastLeftOutput() {
		return leftOutput;
	}

	/**
	 * Retrieves the last output written to the right motor.
	 * 
	 * @return The last right output
	 */
	public double lastRightOutput() {
		return rightOutput;
	}

	/**
	 * Retrieves the last desired (not actual!) velocity of the left wheel.
	 * <p>
	 * This value is multiplied by <b>kV</b>, and added to the left output.
	 * </p>
	 * 
	 * @deprecated Use {@link #lastMoment()} instead
	 * @return The last left velocity
	 * @see #lastMoment()
	 */
	@Deprecated
	public double lastLeftVelocity() {
		return lastMoment.getLeftVelocity();
	}

	/**
	 * Retrieves the last desired (not actual!) velocity of the right wheel.
	 * <p>
	 * This value is multiplied by <b>kV</b>, and added to the left output.
	 * </p>
	 * 
	 * @deprecated Use {@link #lastMoment()} instead
	 * @return The last right velocity
	 * @see #lastMoment()
	 */
	@Deprecated
	public double lastRightVelocity() {
		return lastMoment.getRightVelocity();
	}

	/**
	 * Retrieves the last desired (not actual!) acceleration of the left wheel.
	 * <p>
	 * This value is multiplied by <b>kA</b>, and added to the left output.
	 * </p>
	 * 
	 * @deprecated Use {@link #lastMoment()} instead
	 * @return The last left acceleration
	 * @see #lastMoment()
	 */
	@Deprecated
	public double lastLeftAcceleration() {
		return lastMoment.getLeftAcceleration();
	}

	/**
	 * Retrieves the last desired (not actual!) acceleration of the right wheel.
	 * <p>
	 * This value is multiplied by <b>kA</b>, and added to the left output.
	 * </p>
	 * 
	 * @deprecated Use {@link #lastMoment()} instead
	 * @return The last right acceleration
	 * @see #lastMoment()
	 */
	@Deprecated
	public double lastRightAcceleration() {
		return lastMoment.getRightAcceleration();
	}

	/**
	 * Retrieves the last moment retrieved from the target followable that this
	 * follower tried to follow.
	 * 
	 * @return The last moment retrieved from the target
	 */
	public TankDriveMoment lastMoment() {
		return lastMoment;
	}
}
