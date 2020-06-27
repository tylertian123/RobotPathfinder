package com.arctos6135.robotpathfinder.core.trajectory;

import java.util.Objects;

/**
 * This class contains information about a basic robot (see the docs for
 * {@link BasicTrajectory} for definition) at a moment in time.
 * <p>
 * Moment objects contain information about the position, velocity, acceleration
 * and direction of a robot at a certain time. They're returned by trajectories
 * and motion profiles when querying a specific time.
 * </p>
 * <h2>Difference Between Heading and Facing</h2>
 * <p>
 * <em>Heading</em> refers to the direction <em>the robot is moving in</em>,
 * while <em>facing</em> refers to the direction <em>the front of the robot is
 * facing</em>. Because RobotPathfinder allows the robot to move backwards,
 * these are not necessarily the same. For example, a robot moving backwards
 * would have a heading going backwards, but the facing direction would still be
 * the front.
 * <h3>Relative And Absolute Directions</h3>
 * <p>
 * <em>Absolute</em> directions are directions relative to the positive x-axis,
 * while <em>relative</em> directions are relative to the starting position of
 * the robot. For example, a robot starting in direction &pi;/2 and is currently
 * facing the direction 0 would have an absolute facing direction of 0, but a
 * relative facing direction of -&pi;.
 * </p>
 * 
 * @author Tyler Tian
 * @see Moment
 * @since 3.0.0
 */
public class BasicMoment extends Moment {

	double d, v, a;
	double t;

	@Override
	public boolean equals(Object o) {
		if (o == this)
			return true;
		if (!(o instanceof BasicMoment)) {
			return false;
		}
		BasicMoment m = (BasicMoment) o;
		return heading == m.heading && initialFacing == m.initialFacing && backwards == m.backwards && d == m.d
				&& v == m.v && a == m.a && t == m.t;
	}

	@Override
	public int hashCode() {
		return Objects.hash(heading, initialFacing, backwards, d, v, a, t);
	}

	@Override
	public String toString() {
		return "{" + " position='" + d + "'" + ", velocity='" + v + "'" + ", acceleration='" + a + "'" + ", heading='"
				+ heading + "'" + ", initialFacing='" + initialFacing + "'" + ", backwards='" + backwards + "'"
				+ ", time='" + t + "'" + "}";
	}

	/**
	 * Constructs a new moment with all fields set to 0.s
	 */
	public BasicMoment() {
		d = v = a = t = heading = 0;
	}

	/**
	 * Creates an exact copy of this {@link BasicMoment}.
	 */
	@Override
	public BasicMoment clone() {
		return new BasicMoment(d, v, a, heading, t, initialFacing);
	}

	/**
	 * Creates a new moment with the specified values.
	 * 
	 * @param position     The desired position
	 * @param velocity     The desired velocity
	 * @param acceleration The desired acceleration
	 * @param heading      The desired heading; see the class Javadoc for more
	 *                     information
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading) {
		d = position;
		v = velocity;
		a = acceleration;
		this.heading = heading;
	}

	/**
	 * Creates a new moment with the specified values.
	 * 
	 * @param position     The desired position
	 * @param velocity     The desired velocity
	 * @param acceleration The desired acceleration
	 * @param heading      The desired heading; see the class Javadoc for more
	 *                     information
	 * @param t            The desired time
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading, double t) {
		this(position, velocity, acceleration, heading);
		this.t = t;
	}

	/**
	 * Creates a new moment with the specified values.
	 * 
	 * @param position      The desired position
	 * @param velocity      The desired velocity
	 * @param acceleration  The desired acceleration
	 * @param heading       The desired heading; see the class Javadoc for more
	 *                      information
	 * @param t             The desired time
	 * @param initialFacing The initial direction the robot is <b>facing</b>; see
	 *                      the class Javadoc for more information
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading, double t,
			double initialFacing) {
		this(position, velocity, acceleration, heading);
		this.t = t;
		this.initialFacing = initialFacing;
	}

	/**
	 * Creates a new moment with the specified values.
	 * 
	 * @param position      The desired position
	 * @param velocity      The desired velocity
	 * @param acceleration  The desired acceleration
	 * @param heading       The desired heading; see the class Javadoc for more
	 *                      information
	 * @param t             The desired time
	 * @param initialFacing The initial direction the robot is <b>facing</b>; see
	 *                      the class Javadoc for more information
	 * @param backwards     Whether the robot is driving backwards in this moment
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading, double t,
			double initialFacing, boolean backwards) {
		this(position, velocity, acceleration, heading, t, initialFacing);
		this.backwards = backwards;
	}

	/**
	 * Sets the time of the moment.
	 * 
	 * @param t The new time
	 */
	public void setTime(double t) {
		this.t = t;
	}

	/**
	 * Retrieves the time of the moment.
	 * 
	 * @return The time of the moment
	 */
	public double getTime() {
		return t;
	}

	/**
	 * Sets the position of the moment.
	 * 
	 * @param position The new position
	 */
	public void setPosition(double position) {
		d = position;
	}

	/**
	 * Retrieves the position of the moment.
	 * 
	 * @return The position of the moment
	 */
	public double getPosition() {
		return d;
	}

	/**
	 * Sets the velocity of the moment.
	 * 
	 * @param velocity The new velocity
	 */
	public void setVelocity(double velocity) {
		v = velocity;
	}

	/**
	 * Retrieves the velocity of the moment.
	 * 
	 * @return The velocity
	 */
	public double getVelocity() {
		return v;
	}

	/**
	 * Sets the acceleration of the moment.
	 * 
	 * @param acceleration The new acceleration
	 */
	public void setAcceleration(double acceleration) {
		a = acceleration;
	}

	/**
	 * Retrieves the acceleration of the moment.
	 * 
	 * @return The acceleration of the moment
	 */
	public double getAcceleration() {
		return a;
	}
}
