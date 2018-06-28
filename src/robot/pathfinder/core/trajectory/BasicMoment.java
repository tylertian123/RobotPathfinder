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
	 * @param heading The desired heading; see {@link #getHeading()} and {@link #getFacing()} for more information.
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
	 * @param heading The desired heading; see {@link #getHeading()} and {@link #getFacing()} for more information.
	 * @param t The desired time
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
	 * Retrieves the position of the moment.
	 * @return The position of the moment
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
	 * Retrieves the velocity of the moment.
	 * @return The velocity
	 */
	public double getVelocity() {
		return v;
	}
	/**
	 * Sets the acceleration of the moment.
	 * @param acceleration The new acceleration
	 */
	public void setAcceleration(double acceleration) {
		a = acceleration;
	}
	/**
	 * Retrieves the acceleration of the moment.
	 * @return The acceleration of the moment
	 */
	public double getAcceleration() {
		return a;
	}
	
	/**
	 * Sets the heading of the moment.
	 * <p>
	 * Headings are the direction of the velocity vector; that is, <em><b>they're the direction the robot is moving
	 * in, not the direction the robot is facing</b></em>. For the direction that the robot is <em>facing</em>,
	 * use {@link #getFacing()} instead.
	 * </p>
	 * <p>
	 * The angles are in radians and follow the unit circle, that is, increasing counterclockwise. <em><b>They're
	 * relative to the positive x axis, not the initial direction of the robot</b></em>. For example, if the
	 * first waypoint used to generate a trajectory has a heading of pi/2, then the angle that represents "forwards"
	 * is also pi/2.
	 * </p>
	 * @param heading The new heading
	 */
	public void setHeading(double heading) {
		this.heading = heading;
	}
	/**
	 * Retrieves the heading of the moment. <em>Not to be confused with {@link #getFacing()}.</em>
	 * <p>
	 * Headings are the direction of the velocity vector; that is, <em><b>they're the direction the robot is moving
	 * in, not the direction the robot is facing</b></em>. For the direction that the robot is <em>facing</em>,
	 * use {@link #getFacing()} instead.
	 * </p>
	 * <p>
	 * The angles are in radians and follow the unit circle, that is, increasing counterclockwise. <em><b>They're
	 * relative to the positive x axis, not the initial direction of the robot</b></em>. For example, if the
	 * first waypoint used to generate a trajectory has a heading of pi/2, then the angle that represents "forwards"
	 * is also pi/2.
	 * </p>
	 * @return The heading of the robot at this moment
	 */
	public double getHeading() {
		return heading;
	}
	/**
	 * Retrieves the direction the robot is <em>facing</em> at this moment in time. <em>Not to be confused with
	 * {@link #getHeading()}.</em>
	 * <p>
	 * The angles are in radians and follow the unit circle, that is, increasing counterclockwise. <em><b>They're
	 * relative to the positive x axis, not the initial direction of the robot</b></em>. For example, if the
	 * first waypoint used to generate a trajectory has a heading of pi/2, then the angle that represents "forwards"
	 * is also pi/2.
	 * </p>
	 * <p>
	 * This value is calculated from the heading of the robot and cannot be set directly. It is implemented by
	 * returning the heading when the velocity is positive, and returning the negative of the heading when the 
	 * velocity is negative.
	 * </p>
	 * @return
	 */
	public double getFacing() {
		return v >= 0 ? heading : -heading;
	}
 }
