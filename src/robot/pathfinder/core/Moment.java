package robot.pathfinder.core;

import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * Represents a moment in time of a trajectory; this class contains the time, as well as the desired position,
 * velocity, and acceleration at that point in time.
 * @author Tyler Tian
 *
 */
final public class Moment {
	
	double d, v, a;
	double t;
	
	double pathT;
	
	public Moment() {
		d = v = a = t = pathT = 0;
	}
	
	/**
	 * Creates a new moment with the specified values.
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 */
	public Moment(double position, double velocity, double acceleration) {
		d = position;
		v = velocity;
		a = acceleration;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 * @param t The point in time this moment is located
	 */
	public Moment(double position, double velocity, double acceleration, double t) {
		this(position, velocity, acceleration);
		this.t = t;
	}
	
	/**
	 * Sets the time of the moment.
	 * @param t The new time
	 */
	public void setTime(double t) {
		this.t = t;
	}
	/**
	 * Retrieves the time of the moment.
	 * @return The time of the moment
	 */
	public double getTime() {
		return t;
	}
	/**
	 * Sets the position of the moment.
	 * @param position The new position
	 */
	public void setPosition(double position) {
		d = position;
	}
	/**
	 * Retrieves the desired position of the moment.
	 * @return The desired position of the moment
	 */
	public double getPosition() {
		return d;
	}
	/**
	 * Sets the velocity of the moment.
	 * @param velocity The new velocity
	 */
	public void setVelocity(double velocity) {
		v = velocity;
	}
	/**
	 * Retrieves the desired velocity of the moment.
	 * @return The desired velocity
	 */
	public double getVelocity() {
		return v;
	}
	/**
	 * Sets the acceleration of the moment
	 * @param acceleration The new acceleration
	 */
	public void setAcceleration(double acceleration) {
		a = acceleration;
	}
	/**
	 * Retrieves the desired acceleration of the moment
	 * @return The desired acceleration of the moment
	 */
	public double getAcceleration() {
		return a;
	}
	
	public void zz_setPathT(double pathT, BasicTrajectory.MomentKey key) {
		if(key == null) {
			throw new RuntimeException("Access denied");
		}
		this.pathT = pathT;
	}
	public double zz_getPathT(TankDriveTrajectory.MomentKey key) {
		if(key == null) {
			throw new RuntimeException("Access denied");
		}
		return pathT;
	}
	double R;
	public void setR(double R) {
		this.R = R;
	}
	public double getR() {
		return R;
	}
}
