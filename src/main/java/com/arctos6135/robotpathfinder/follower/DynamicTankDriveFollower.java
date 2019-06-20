package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.math.MathUtils;

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
	 * Sets the directional-proportional gain of the feedback loop.
	 * <p>
	 * The directional-proportional gain allows the robot to better follow the
	 * trajectory by trying to follow not just the position, velocity and
	 * acceleration, but the direction as well. The actual angle the robot is facing
	 * at a given time is subtracted from the angle it is supposed to be facing, and
	 * then multiplied by the directional-proportional gain and added/subtracted to
	 * the outputs of the left and right wheels.
	 * </p>
	 * 
	 * @param kDP The new directional-proportional gain
	 */
	public void setDP(double kDP) {
		this.kDP = kDP;
	}

	/**
	 * Gets the directional-proportional gain of the feedback loop.
	 * <p>
	 * The directional-proportional gain allows the robot to better follow the
	 * trajectory by trying to follow not just the position, velocity and
	 * acceleration, but the direction as well. The actual angle the robot is facing
	 * at a given time is subtracted from the angle it is supposed to be facing, and
	 * then multiplied by the directional-proportional gain and added/subtracted to
	 * the outputs of the left and right wheels.
	 * </p>
	 * 
	 * @return The directional-proportional gain
	 */
	public double getDP() {
		return kDP;
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
	 */
	public void setGains(double kV, double kA, double kP, double kI, double kD, double kDP) {
		setGains(kV, kA, kP, kI, kD);
		setDP(kDP);
	}

	/**
	 * Sets the timestamp source.
	 * 
	 * @param timer The new timestamp source
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
	 * Sets the distance sources.
	 * 
	 * @param lDistSrc The left distance source
	 * @param rDistSrc The right distance source
	 * @throws IllegalStateException If the follower is running
	 */
	public void setDistanceSources(AdvancedPositionSource lDistSrc, AdvancedPositionSource rDistSrc) {
		if (running) {
			throw new IllegalStateException("Distance Sources cannot be changed when follower is running");
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
	 * @return The last left velocity
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
	 * @return The last right velocity
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
	 * @return The last left acceleration
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
	 * @return The last right acceleration
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
