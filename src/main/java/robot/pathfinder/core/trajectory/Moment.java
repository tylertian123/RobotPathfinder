package robot.pathfinder.core.trajectory;

/**
 * A class that holds information about a robot at a moment in time. 
 * <p>
 * Moment objects contain information about the position, velocity, acceleration and direction of a robot
 * at a certain time. They're returned by trajectories when querying a specific time.
 * </p>
 * <p>
 * Note that because different robot types call for drastically different moment implementations,
 * there are very few methods that can be shared between all of them.
 * </p>
 * <h2>Difference Between Heading and Facing</h2>
 * <p>
 * <em>Heading</em> refers to the direction <em>the robot is moving in</em>, while <em>facing</em> refers to
 * the direction <em>the front of the robot is facing</em>. Because RobotPathfinder allows the robot to move 
 * backwards, these are not necessarily the same. For example, a robot moving backwards would have a heading
 * going backwards, but the facing direction would still be the front.
 * <h3>Relative And Absolute Directions</h3>
 * <p>
 * <em>Absolute</em> directions are directions relative to the positive x-axis, while <em>relative</em> directions
 * are relative to the starting position of the robot. For example, a robot starting in direction &pi;/2 and
 * is currently facing the direction 0 would have an absolute facing direction of 0, but a relative 
 * facing direction of -&pi;.
 * </p>
 * <h2>Units</h2>
 * <p>
 * The units used for these moment objects are completely decided by which units are used in a trajectory's
 * {@link robot.pathfinder.core.RobotSpecs RobotSpecs} during generation. For example, if the unit for max velocity was in m/s, then the unit used
 * here for velocity would also be m/s. All angles are in radians and are in the range (-&pi;, &pi;].
 * </p>
 * @author Tyler Tian
 *
 */
public abstract class Moment implements Cloneable {
	
	double heading;

	/**
	 * Retrieves the direction the robot is moving in. 
	 * For more information, see the class JavaDoc.
	 * @return The heading of the robot
	 */
	public double getHeading() {
		return heading;
	}
	/**
	 * Sets the direction the robot is moving in.
	 * For more information, see the class JavaDoc.
	 * @param heading The new heading of the robot
	 */
	public void setHeading(double heading) {
		this.heading = heading;
	}
	
	/**
	 * Retrieves the <em>initial</em> direction the robot is <em>facing</em>. This value is used to calculate
	 * the result of {@link #getFacingRelative()}. For more information, see the class JavaDoc.
	 * @return The initial direction the robot is facing
	 */
	abstract public double getInitialFacing();
	/**
	 * Sets the <em>initial</em> direction the robot is <em>facing</em>. This value is used to calculate
	 * the result of {@link #getFacingRelative()}. For more information, see the class JavaDoc.
	 * @param initialFacing The initial direction the robot is facing
	 */
	abstract public void setInitialFacing(double initialFacing);
	/**
	 * Retrieves the direction the robot is facing, relative to the starting position of the robot.
	 * For more information, see the class JavaDoc.
	 * @return The relative facing direction of the robot
	 */
	abstract public double getFacingRelative();
	/**
	 * Retrieves the direction the robot is facing, relative to the positive x-axis.
	 * For more information, see the class JavaDoc.
	 * @return The absolute facing direction of the robot
	 */
	abstract public double getFacingAbsolute();
}
