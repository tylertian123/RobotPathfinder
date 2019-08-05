package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveGains;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveRobot;
import com.arctos6135.robotpathfinder.math.MathUtils;

/**
 * A dynamic follower class for tank drive robots.
 * <p>
 * See {@link DynamicFollower} for the basic definition of a "dynamic follower".
 * This class provides a concrete implementation of the abstract class
 * {@link DynamicFollower} for {@link TankDriveMoment}.
 * </p>
 * <p>
 * This class is essentially a {@link DynamicFollower} version of
 * {@link TankDriveFollower}. It contains every method of
 * {@link TankDriveFollower} except for the constructors and those inherited
 * from {@link DynamicFollower}. Note that it does not inherit from
 * {@link TankDriveFollower} since multiple inheritance is not allowed.
 * </p>
 * <p>
 * However, one major difference between this class and
 * {@link TankDriveFollower} is that this class requires not just the wheel
 * positions, but also their velocities and accelerations. Therefore, it uses
 * {@link Follower.AdvancedPositionSource}s, which provide position, velocity
 * and acceleration data. Nevertheless, if provided with a regular
 * {@link Follower.PositionSource}, it will attempt to calculate the velocity
 * and acceleration manually by differentiating the position. Therefore, all
 * constructors require a {@link PositionSource} of some kind.
 * </p>
 * <p>
 * See the documentation for {@link Follower} for usage instructions.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 * @see TankDriveFollower
 * @see DynamicFollowable
 */
public class DynamicTankDriveFollower extends DynamicFollower<TankDriveMoment> {

	protected Motor lMotor, rMotor;
	// Not null
	// Either a regular DistanceSource or an AdvancedDistanceSource
	protected PositionSource lDistSrc, rDistSrc;
	// This variable keeps track of whether the position sources are advanced
	// If true they will directly be used
	// If false the velocity and acceleration will be calculated manually
	protected boolean advancedDistSrc = false;
	// Can be null
	protected DirectionSource directionSrc;

	// Directional proportional gain
	protected double kDP = 0;

	// Keep track of the initial timestamp and distance measurements so we don't
	// have to reset
	// Keep track of the error and timestamp of the last iteration to calculate the
	// derivative
	protected double initTime, lastTime, lLastErr, rLastErr, lInitDist, rInitDist, initDirection, lErrorInt, rErrorInt;

	// Used for finding the velocity and acceleration for updating if a regular
	// DistanceSource is used
	protected double lLastPos, rLastPos, lLastVel, rLastVel, lLastAccel, rLastAccel;

	protected double leftErr, rightErr, dirErr;
	// Store these as member variables so they can be accessed from outside the
	// class for testing purposes
	protected double leftOutput, rightOutput, leftDeriv, rightDeriv;
	protected TankDriveMoment lastMoment;

	/*
	 * Note that one of the differences between this class and TankDriveFollower is
	 * the lack of the two constructors that have no distance source. Since updating
	 * the trajectory requires that we know the distance, the two constructors were
	 * removed.
	 */

	/**
	 * Constructs a new dynamic tank drive follower using
	 * {@link AdvancedPositionSource}s and no DP term.
	 * <p>
	 * This constructor does not require any sensors for direction, or a value for
	 * kDP. Therefore, the directional-proportional term is not used. This may
	 * reduce accuracy, but is typically good enough.
	 * </p>
	 * 
	 * @param target      The target to follow
	 * @param lMotor      The left side motor
	 * @param rMotor      The right side motor
	 * @param lDistSrc    A {@link AdvancedPositionSource} for the left motor
	 * @param rDistSrc    A {@link AdvancedPositionSource} for the right motor
	 * @param timer       A {@link TimestampSource} to get the current time from
	 * @param kV          The velocity feedforward
	 * @param kA          The acceleration feedforward
	 * @param kP          The proportional gain
	 * @param kI          The integral gain
	 * @param kD          The derivative gain
	 * @param updateDelay The duration between two updates
	 * @deprecated Use
	 *             {@link #DynamicTankDriveFollower(DynamicFollowable, TankDriveRobot, Gains, double)}
	 *             instead.
	 */
	@Deprecated
	public DynamicTankDriveFollower(DynamicFollowable<TankDriveMoment> target, Motor lMotor, Motor rMotor,
			AdvancedPositionSource lDistSrc, AdvancedPositionSource rDistSrc, TimestampSource timer, double kV,
			double kA, double kP, double kI, double kD, double updateDelay) {
		setGains(kV, kA, kP, kI, kD, 0);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = null;
		this.updateDelay = updateDelay;
		advancedDistSrc = true;
	}

	/**
	 * Constructs a new dynamic tank drive follower using
	 * {@link AdvancedPositionSource}s.
	 * 
	 * @param target      The target to follow
	 * @param lMotor      The left side motor
	 * @param rMotor      The right side motor
	 * @param lDistSrc    A {@link AdvancedPositionSource} for the left motor
	 * @param rDistSrc    A {@link AdvancedPositionSource} for the right motor
	 * @param timer       A {@link TimestampSource} to get the current time from
	 * @param dirSrc      A {@link DirectionSource} to get angle data from
	 * @param kV          The velocity feedforward
	 * @param kA          The acceleration feedforward
	 * @param kP          The proportional gain
	 * @param kI          The integral gain
	 * @param kD          The derivative gain
	 * @param kDP         The directional-proportional gain
	 * @param updateDelay The duration between two updates
	 * @deprecated Use
	 *             {@link #DynamicTankDriveFollower(DynamicFollowable, TankDriveRobot, Gains, double)}
	 *             instead.
	 */
	@Deprecated
	public DynamicTankDriveFollower(DynamicFollowable<TankDriveMoment> target, Motor lMotor, Motor rMotor,
			AdvancedPositionSource lDistSrc, AdvancedPositionSource rDistSrc, TimestampSource timer,
			DirectionSource dirSrc, double kV, double kA, double kP, double kI, double kD, double kDP,
			double updateDelay) {
		setGains(kV, kA, kP, kI, kD, kDP);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = dirSrc;
		this.updateDelay = updateDelay;
		advancedDistSrc = true;
	}

	/**
	 * Constructs a new dynamic tank drive follower using regular
	 * {@link PositionSource}s and no DP term.
	 * <p>
	 * This constructor uses regular {@link PositionSource}s instead of
	 * {@link AdvancedPositionSource}s, and obtains the velocity and acceleration
	 * through manual differentiation instead.
	 * </p>
	 * <p>
	 * It also does not require any sensors for direction, or a value for kDP.
	 * Therefore, the directional-proportional term is not used. This may reduce
	 * accuracy, but is typically good enough.
	 * </p>
	 * 
	 * @param target      The target to follow
	 * @param lMotor      The left side motor
	 * @param rMotor      The right side motor
	 * @param lDistSrc    A {@link PositionSource} for the left motor
	 * @param rDistSrc    A {@link PositionSource} for the right motor
	 * @param timer       A {@link TimestampSource} to get the current time from
	 * @param kV          The velocity feedforward
	 * @param kA          The acceleration feedforward
	 * @param kP          The proportional gain
	 * @param kI          The integral gain
	 * @param kD          The derivative gain
	 * @param updateDelay The duration between two updates
	 * @deprecated Use
	 *             {@link #DynamicTankDriveFollower(DynamicFollowable, TankDriveRobot, Gains, double)}
	 *             instead.
	 */
	@Deprecated
	public DynamicTankDriveFollower(DynamicFollowable<TankDriveMoment> target, Motor lMotor, Motor rMotor,
			PositionSource lDistSrc, PositionSource rDistSrc, TimestampSource timer, double kV, double kA, double kP,
			double kI, double kD, double updateDelay) {
		setGains(kV, kA, kP, kI, kD, 0);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = null;
		this.updateDelay = updateDelay;
		advancedDistSrc = false;
	}

	/**
	 * Constructs a new dynamic tank drive follower using
	 * {@link AdvancedPositionSource}s.
	 * <p>
	 * This constructor uses regular {@link PositionSource}s instead of
	 * {@link AdvancedPositionSource}s, and obtains the velocity and acceleration
	 * through manual differentiation instead.
	 * </p>
	 * 
	 * @param target      The target to follow
	 * @param lMotor      The left side motor
	 * @param rMotor      The right side motor
	 * @param lDistSrc    A {@link PositionSource} for the left motor
	 * @param rDistSrc    A {@link PositionSource} for the right motor
	 * @param timer       A {@link TimestampSource} to get the current time from
	 * @param dirSrc      A {@link DirectionSource} to get angle data from
	 * @param kV          The velocity feedforward
	 * @param kA          The acceleration feedforward
	 * @param kP          The proportional gain
	 * @param kI          The integral gain
	 * @param kD          The derivative gain
	 * @param kDP         The directional-proportional gain
	 * @param updateDelay The duration between two updates
	 * @deprecated Use
	 *             {@link #DynamicTankDriveFollower(DynamicFollowable, TankDriveRobot, Gains, double)}
	 *             instead.
	 */
	@Deprecated
	public DynamicTankDriveFollower(DynamicFollowable<TankDriveMoment> target, Motor lMotor, Motor rMotor,
			PositionSource lDistSrc, PositionSource rDistSrc, TimestampSource timer, DirectionSource dirSrc, double kV,
			double kA, double kP, double kI, double kD, double kDP, double updateDelay) {
		setGains(kV, kA, kP, kI, kD, kDP);
		this.target = target;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
		this.directionSrc = dirSrc;
		this.updateDelay = updateDelay;
		advancedDistSrc = false;
	}

	/**
	 * Constructs a new dynamic tank drive follower.
	 * <p>
	 * Note that ideally {@link TankDriveRobot#leftPositionSource
	 * robot.leftPositionSource} and {@link TankDriveRobot#rightPositionSource
	 * robot.rightPositionSource} should be instances of
	 * {@link AdvancedPositionSource}, since updating the target followable often
	 * requires knowing the current velocity and acceleration in addition to
	 * position. However, if they're not instances of
	 * {@link AdvancedPositionSource}, this follower will attempt to derive the
	 * velocity and acceleration manually from the position data, which may not be
	 * as efficient.
	 * </p>
	 * <p>
	 * If any of {@link TankDriveRobot#leftMotor robot.leftMotor},
	 * {@link TankDriveRobot#rightMotor robot.rightMotor},
	 * {@link TankDriveRobot#leftPositionSource robot.leftPositionSource}
	 * {@link TankDriveRobot#rightPositionSource robot.rightPositionSource}, or
	 * {@link TankDriveRobot#timestampSource robot.timestampSource} is {@code null},
	 * this constructor will throw an {@link IllegalArgumentException}.
	 * </p>
	 * <p>
	 * If {@link TankDriveRobot#directionSource robot.directionSource} is
	 * {@code null}, the directional-proportional term will not be used.
	 * </p>
	 * <p>
	 * If {@code updateDelay} is {@code NaN}, the target will never be updated.
	 * </p>
	 * 
	 * @param target      The target {@link Followable} to follow
	 * @param robot       A {@link TankDriveRobot} object containing the necessary
	 *                    motors and sensors
	 * @param gains       A set of all the gains used in the control loop
	 * @param updateDelay The duration between two updates of the target; set to
	 *                    {@code NaN} to disable updating
	 */
	public DynamicTankDriveFollower(DynamicFollowable<TankDriveMoment> target, TankDriveRobot robot, Gains gains,
			double updateDelay) {
		setGains(gains);
		this.target = target;

		// Verify motors are nonnull
		if (robot.leftMotor == null || robot.rightMotor == null) {
			throw new IllegalArgumentException("Motors cannot be null!");
		}
		// Verify timer is nonnull
		if (robot.timestampSource == null) {
			throw new IllegalArgumentException("Timestamp source cannot be null!");
		}
		// Verify position sources are nonnull
		if (robot.leftPositionSource == null || robot.rightPositionSource == null) {
			throw new IllegalArgumentException("Position sources cannot be null!");
		}
		lMotor = robot.leftMotor;
		rMotor = robot.rightMotor;
		lDistSrc = robot.leftPositionSource;
		rDistSrc = robot.rightPositionSource;
		timer = robot.timestampSource;
		directionSrc = robot.directionSource;
		this.updateDelay = updateDelay;

		advancedDistSrc = (lDistSrc instanceof AdvancedPositionSource) && (rDistSrc instanceof AdvancedPositionSource);
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
	 * @deprecated This method should not be used. Create a new object instead.
	 */
	@Deprecated
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
	 * @deprecated This method should not be used. Create a new object instead.
	 */
	@Deprecated
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
	 * @deprecated This method should not be used. Create a new object instead.
	 */
	@Deprecated
	public void setDistanceSources(AdvancedPositionSource lDistSrc, AdvancedPositionSource rDistSrc) {
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

		// If using manual calculations of the velocity and acceleration, reset the
		// variables used
		if (!advancedDistSrc) {
			lLastPos = lInitDist;
			rLastPos = rInitDist;
			lLastVel = rLastVel = lLastAccel = rLastAccel = 0;
		}
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
		// Calculate errors and derivatives
		// No need for null check
		// Calculate left and right errors
		double lPos = (lDistSrc.getPosition() - lInitDist);
		double rPos = (rDistSrc.getPosition() - rInitDist);
		leftErr = m.getLeftPosition() - lPos;
		rightErr = m.getRightPosition() - rPos;
		// Get the derivative of the errors
		leftDeriv = (leftErr - lLastErr) / dt;
		rightDeriv = (rightErr - rLastErr) / dt;
		// Calculate the integral of the error
		lErrorInt += leftErr * dt;
		rErrorInt += rightErr * dt;

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

		if (!advancedDistSrc) {
			double lVel = (lPos - lLastPos) / dt;
			double rVel = (rPos - rLastPos) / dt;
			lLastPos = lPos;
			rLastPos = rPos;

			lLastAccel = (lVel - lLastVel) / dt;
			rLastAccel = (rVel - rLastVel) / dt;
			lLastVel = lVel;
			rLastVel = rVel;
		}

		return false;
	}

	@Override
	protected void _stop() {
		lMotor.set(0);
		rMotor.set(0);
	}

	@Override
	protected void _update() {
		// To construct the moment used to update the trajectory, we need to get the
		// heading
		// Since implementations of the update method mainly use the relative facing,
		// we'll start with that
		double facing;
		// Grab the relative facing
		// The direction source can be null, so check here
		if (directionSrc == null) {
			// If null, set the heading to be the last desired heading
			facing = lastMoment.getFacingRelative();
		} else {
			// Otherwise grab the actual direction
			// Remember to subtract the initial direction to obtain the relative facing
			facing = directionSrc.getDirection() - initDirection;
		}
		// Now calculate the heading to be passed into the constructor
		// First, add the initial facing to get the absolute facing
		double heading = facing + lastMoment.getInitialFacing();
		// If the moment is reversed, the heading is negated to get the facing
		// Therefore we negate it to find the heading
		if (lastMoment.getBackwards()) {
			heading = -heading;
		}

		// Get the positions, velocities and accelerations
		double ld;
		double rd;
		double lv;
		double rv;
		double la;
		double ra;
		// If using advanced position sources, just cast them and call the methods
		if (advancedDistSrc) {
			ld = ((AdvancedPositionSource) lDistSrc).getPosition();
			rd = ((AdvancedPositionSource) rDistSrc).getPosition();
			lv = ((AdvancedPositionSource) lDistSrc).getVelocity();
			rv = ((AdvancedPositionSource) rDistSrc).getVelocity();
			la = ((AdvancedPositionSource) lDistSrc).getAcceleration();
			ra = ((AdvancedPositionSource) rDistSrc).getAcceleration();
		}
		// Otherwise use the calculated values
		else {
			ld = lLastPos;
			rd = rLastPos;
			lv = lLastVel;
			rv = rLastVel;
			la = lLastAccel;
			ra = rLastAccel;
		}

		// Finally construct the moment and update the trajectory
		// Since constructors all take DynamicFollowables, there is no risk of
		// ClassCastException
		((DynamicFollowable<TankDriveMoment>) target).update(new TankDriveMoment(ld, rd, lv, rv, la, ra, heading,
				timer.getTimestamp(), lastMoment.getInitialFacing(), lastMoment.getBackwards()));
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
