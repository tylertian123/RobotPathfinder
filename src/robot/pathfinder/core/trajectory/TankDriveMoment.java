package robot.pathfinder.core.trajectory;

/**
 * A class that holds information about a tank drive robot at a moment in time. Unlike {@link BasicMoment},
 * this class holds information about two wheels.
 * <p>
 * Moment objects contain information about the position, velocity, acceleration and direction of a robot
 * at a certain time. They're returned by trajectories when querying a specific time.
 * </p>
 * <p>
 * Note that the units used for these moment objects are completely decided by which units are used in a trajectory's
 * {@link robot.pathfinder.core.RobotSpecs RobotSpecs} during generation. For example, if the unit for max velocity was in m/s, then the unit used
 * here for velocity would also be m/s.
 * </p>
 * @author Tyler Tian
 *
 */
public class TankDriveMoment implements Moment {
	
	double ld, lv, la, rd, rv, ra;
	double t;
	double heading;
	
	double initialFacing = 0;
	
	/**
	 * Constructs a new moment with all fields set to 0.s
	 */
	public TankDriveMoment() {
		ld = lv = la = rd = rv = ra = t = heading = 0;
	}
	
	@Override
	public TankDriveMoment clone() {
		return new TankDriveMoment(ld, rd, lv, rv, la, ra, heading, t);
	}
	
	/**
	 * Creates a new moment with the specified values.
	 * @param leftPos The distance the left wheel has traveled
	 * @param rightPos The distance the right wheel has traveled
	 * @param leftVel The velocity of the left wheel
	 * @param rightVel The velocity of the right wheel
	 * @param leftAcl The acceleration of the left wheel
	 * @param rightAcl The acceleration of the right wheel
	 * @param heading The desired heading; see {@link #getHeading()} and {@link #getFacing()} for more information.
	 */
	public TankDriveMoment(double leftPos, double rightPos, double leftVel, double rightVel, double leftAcl, double rightAcl, double heading) {
		ld = leftPos;
		rd = rightPos;
		lv = leftVel;
		rv = rightVel;
		la = leftAcl;
		ra = rightAcl;
		this.heading = heading;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param leftPos The distance the left wheel has traveled
	 * @param rightPos The distance the right wheel has traveled
	 * @param leftVel The velocity of the left wheel
	 * @param rightVel The velocity of the right wheel
	 * @param leftAcl The acceleration of the left wheel
	 * @param rightAcl The acceleration of the right wheel
	 * @param heading The desired heading; see {@link #getHeading()} and {@link #getFacing()} for more information.
	 * @param time The desired time
	 */
	public TankDriveMoment(double leftPos, double rightPos, double leftVel, double rightVel, double leftAcl, double rightAcl, double heading, double time) {
		this(leftPos, rightPos, leftVel, rightVel, leftAcl, rightAcl, heading);
		t = time;
	}
	/**
	 * Creates a new moment with the specified values.
	 * @param leftPos The distance the left wheel has traveled
	 * @param rightPos The distance the right wheel has traveled
	 * @param leftVel The velocity of the left wheel
	 * @param rightVel The velocity of the right wheel
	 * @param leftAcl The acceleration of the left wheel
	 * @param rightAcl The acceleration of the right wheel
	 * @param heading The desired heading; see {@link #getHeading()} and {@link #getFacing()} for more information
	 * @param time The desired time
	 * @param initialFacing The initial direction the robot is facing; used to calculate the result of {@link #getFacingRelative()}
	 */
	public TankDriveMoment(double leftPos, double rightPos, double leftVel, double rightVel, double leftAcl, double rightAcl, double heading, double time, double initialFacing) {
		this(leftPos, rightPos, leftVel, rightVel, leftAcl, rightAcl, heading);
		t = time;
		this.initialFacing = initialFacing;
	}

	/**
	 * Retrieves the distance the left wheel has traveled.
	 * @return The distance of the left wheel
	 */
	public double getLeftPosition() {
		return ld;
	}

	/**
	 * Sets the distance the left wheel has traveled.
	 * @param ld The new distance of the left wheel
	 */
	public void setLeftPosition(double ld) {
		this.ld = ld;
	}

	/**
	 * Retrieves the velocity of the left wheel.
	 * @return The velocity of the left wheel
	 */
	public double getLeftVelocity() {
		return lv;
	}

	/**
	 * Sets the velocity of the left wheel.
	 * @param lv The new velocity of the left wheel
	 */
	public void setLeftVelocity(double lv) {
		this.lv = lv;
	}

	/**
	 * Retrieves the acceleration of the left wheel.
	 * @return The acceleration of the left wheel
	 */
	public double getLeftAcceleration() {
		return la;
	}

	/**
	 * Sets the acceleration of the left wheel.
	 * @param la The new acceleration of the left wheel
	 */
	public void setLeftAcceleration(double la) {
		this.la = la;
	}

	/**
	 * Retrieves the distance traveled by the right wheel.
	 * @return The distance traveled by the right wheel
	 */
	public double getRightPosition() {
		return rd;
	}

	/**
	 * Sets the distance traveled by the right wheel.
	 * @param rd The new distance traveled by the right wheel.
	 */
	public void setRightPosition(double rd) {
		this.rd = rd;
	}

	/**
	 * Retrieves the velocity of the right wheel.
	 * @return The velocity of the right wheel
	 */
	public double getRightVelocity() {
		return rv;
	}

	/**
	 * Sets the velocity of the right wheel.
	 * @param rv The new velocity of the right wheel
	 */
	public void setRightVelocity(double rv) {
		this.rv = rv;
	}

	/**
	 * Retrieves the acceleration of the right wheel.
	 * @return The acceleration of the right wheel
	 */
	public double getRightAcceleration() {
		return ra;
	}

	/**
	 * Sets the acceleration of the right wheel.
	 * @param ra The new acceleration of the right wheel
	 */
	public void setRightAcceleration(double ra) {
		this.ra = ra;
	}

	/**
	 * Retrieves the time.
	 * @return The time
	 */
	public double getTime() {
		return t;
	}

	/**
	 * Sets the time.
	 * @param t The new time
	 */
	public void setTime(double t) {
		this.t = t;
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
	
	public double getInitialFacing() {
		return initialFacing;
	}
	public void setInitialFacing(double initFacing) {
		initialFacing = initFacing;
	}
	@Override
	public double getFacingRelative() {
		return getFacingAbsolute() - initialFacing;
	}
	@Override
	public double getFacingAbsolute() {
		return lv >= 0 || rv >= 0 ? heading : -heading;
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
	 * returning the heading when at least one wheel's velocity is positive, returning the negative of the heading
	 * when both wheels have negative velocities.
	 * </p>
	 * @deprecated Use {@link #getFacingRelative()} or {@link #getFacingAbsolute()} instead.
	 * @return The direction the robot is facing
	 */
	public double getFacing() {
		return lv >= 0 || rv >= 0 ? heading : -heading;
	}
	
	/**
	 * Retrieves information about the left wheel, heading and time, stored in a {@link BasicMoment} object.
	 * @return Information about the left wheel, heading and time, stored in a {@link BasicMoment} object
	 */
	public BasicMoment leftComponent() {
		return new BasicMoment(ld, lv, la, heading, t);
	}
	/**
	 * Retrieves information about the right wheel, heading and time, stored in a {@link BasicMoment} object.
	 * @return Information about the right wheel, heading and time, stored in a {@link BasicMoment} object
	 */
	public BasicMoment rightComponent() {
		return new BasicMoment(rd, rv, ra, heading, t);
	}
	
	/**
	 * Makes a {@link TankDriveMoment} from information about the left and right wheels. Note that the heading of
	 * the new moment is taken from the left component.
	 * @param left Information about the left wheel
	 * @param right Information about the right wheel
	 * @return A new {@link TankDriveMoment} that combines the information from the left and right wheels
	 */
	public static TankDriveMoment fromComponents(BasicMoment left, BasicMoment right) {
		return new TankDriveMoment(left.getPosition(), right.getPosition(), left.getVelocity(), right.getVelocity(),
				left.getAcceleration(), right.getAcceleration(), left.getHeading(), left.getTime());
	}
}
