package robot.pathfinder;

/**
 * Represents a moment in time of a trajectory; this class contains the time, as well as the desired distance,
 * velocity, and acceleration at that point in time.
 * @author Tyler Tian
 *
 */
public class Moment {
	
	double d, v, a;
	double t;
	
	/**
	 * Creates a new moment with the specified values.
	 * @param distance - The desired distance
	 * @param velocity - The desired velocity
	 * @param acceleration - The desired acceleration
	 */
	public Moment(double distance, double velocity, double acceleration) {
		d = distance;
		v = velocity;
		a = acceleration;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param distance - The desired distance
	 * @param velocity - The desired velocity
	 * @param acceleration - The desired acceleration
	 * @param t - The point in time this moment is located
	 */
	public Moment(double distance, double velocity, double acceleration, double t) {
		this(distance, velocity, acceleration);
		this.t = t;
	}
	
	/**
	 * Sets the time of the moment.
	 * @param t - The new time
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
	 * Sets the distance of the moment.
	 * @param distance - The new distance
	 */
	public void setDistance(double distance) {
		d = distance;
	}
	/**
	 * Retrieves the desired distance of the moment.
	 * @return The desired distance of the moment
	 */
	public double getDistance() {
		return d;
	}
	/**
	 * Sets the velocity of the moment.
	 * @param velocity - The new velocity
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
	 * @param acceleration - The new acceleration
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
}
