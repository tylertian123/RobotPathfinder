package robot.pathfinder.core.trajectory;

/**
 * A class that holds information about a robot at a moment in time.
 * <p>
 * Moment objects contain information about the position, velocity, acceleration
 * and direction of a robot at a certain time. They're returned by trajectories
 * when querying a specific time.
 * </p>
 * <p>
 * This class represents a moment in time for a tank drive robot.
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
 * <h2>Units</h2>
 * <p>
 * The units used for these moment objects are completely decided by which units
 * are used in a trajectory's {@link robot.pathfinder.core.RobotSpecs
 * RobotSpecs} during generation. For example, if the unit for max velocity was
 * in m/s, then the unit used here for velocity would also be m/s.
 * </p>
 * 
 * @author Tyler Tian
 *
 */
public class TankDriveMoment extends Moment {
	
	double ld, lv, la, rd, rv, ra;
	double t;
	
	/**
	 * Constructs a new moment with all fields set to 0.
	 */
	public TankDriveMoment() {
		ld = lv = la = rd = rv = ra = t = heading = 0;
	}
	
	@Override
	public TankDriveMoment clone() {
		return new TankDriveMoment(ld, rd, lv, rv, la, ra, heading, t, initialFacing);
	}
	
	/**
	 * Creates a new moment with the specified values.
	 * @param leftPos The distance the left wheel has traveled
	 * @param rightPos The distance the right wheel has traveled
	 * @param leftVel The velocity of the left wheel
	 * @param rightVel The velocity of the right wheel
	 * @param leftAcl The acceleration of the left wheel
	 * @param rightAcl The acceleration of the right wheel
	 * @param heading The desired heading; see the class JavaDoc for more information
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
	 * @param heading The desired heading; see the class JavaDoc for more information
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
	 * @param heading The desired heading; see the class JavaDoc for more information
	 * @param time The desired time
	 * @param initialFacing The initial direction the robot is <b>facing</b>; see the class JavaDoc for more information
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
