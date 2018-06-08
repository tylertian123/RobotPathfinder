package robot.pathfinder.core;

/**
 * Represents a moment in time of a trajectory; this class contains the time, as well as the desired position,
 * velocity, and acceleration at that point in time.
 * @author Tyler Tian
 *
 */
final public class Moment {
	
	double d, v, a, j;
	double t;
	
	boolean locked = false;
	
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
	 * Creates a new moment with the specified values
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 * @param jerk The desired jerk (change in acceleration)
	 */
	public Moment(double position, double velocity, double acceleration, double jerk) {
		d = position;
		v = velocity;
		a = acceleration;
		j = jerk;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 * @param jerk The desired jerk (change in acceleration)
	 * @param t The point in time this moment is located
	 */
	public Moment(double position, double velocity, double acceleration, double jerk, double t) {
		this(position, velocity, acceleration, jerk);
		this.t = t;
	}
	
	/**
	 * Returns whether this object is locked. Attempting to modify a locked object will cause a {@link LockedException} to be thrown.
	 * @return
	 */
	public boolean isLocked() {
		return locked;
	}
	
	/**
	 * Locks the object. Any subsequent attempts to modify will cause a {@link LockedException} to be thrown.
	 */
	public void lock() {
		locked = true;
	}
	
	protected void unlock() {
		locked = false;
	}
	
	/**
	 * Sets the time of the moment.
	 * @param t The new time
	 */
	public void setTime(double t) {
		if(isLocked())
			throw new LockedException("Attempt to modify a locked Moment");
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
		if(isLocked())
			throw new LockedException("Attempt to modify a locked Moment");
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
		if(isLocked())
			throw new LockedException("Attempt to modify a locked Moment");
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
		if(isLocked())
			throw new LockedException("Attempt to modify a locked Moment");
		a = acceleration;
	}
	/**
	 * Retrieves the desired acceleration of the moment
	 * @return The desired acceleration of the moment
	 */
	public double getAcceleration() {
		return a;
	}
	/**
	 * Sets the jerk (change in acceleration) of the moment
	 * @param jerk The desired jerk
	 */
	public void setJerk(double jerk) {
		if(isLocked())
			throw new LockedException("Attempt to modify a locked Moment");
		j = jerk;
	}
	/**
	 * Retrieves the desired jerk (change in acceleration) of the moment
	 * @return The desired jerk of the moment
	 */
	public double getJerk() {
		return j;
	}
}
