package robot.pathfinder.core.trajectory;

/**
 * A class that holds information about a robot at a moment in time. 
 * <p>
 * Moment objects contain information about the position, velocity, acceleration and direction of a robot
 * at a certain time. They're returned by trajectories when querying a specific time.
 * </p>
 * @author Tyler Tian
 *
 */
public class BasicMoment implements Cloneable {
	
	double d, v, a;
	double t;
	double heading;
	
	/**
	 * Constructs a new moment with all fields set to 0.s
	 */
	public BasicMoment() {
		d = v = a = t = heading = 0;
	}
	
	@Override
	public BasicMoment clone() {
		return new BasicMoment(d, v, a, heading, t);
	}
	
	/**
	 * Creates a new moment with the specified values.
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading) {
		d = position;
		v = velocity;
		a = acceleration;
		this.heading = heading;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param position The desired position
	 * @param velocity The desired velocity
	 * @param acceleration The desired acceleration
	 * @param t The point in time this moment is located
	 */
	public BasicMoment(double position, double velocity, double acceleration, double heading, double t) {
		this(position, velocity, acceleration, heading);
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
	
	public void setHeading(double heading) {
		this.heading = heading;
	}
	public double getHeading() {
		return heading;
	}
	public double getFacing() {
		return v >= 0 ? heading : -heading;
	}
 }
