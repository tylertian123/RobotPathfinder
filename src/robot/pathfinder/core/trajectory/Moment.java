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
 * <p>
 * Note that the units used for these moment objects are completely decided by which units are used in a trajectory's
 * {@link robot.pathfinder.core.RobotSpecs RobotSpecs} during generation. For example, if the unit for max velocity was in m/s, then the unit used
 * here for velocity would also be m/s.
 * </p>
 * @author Tyler Tian
 *
 */
public interface Moment extends Cloneable {
	public double getHeading();
	public void setHeading(double heading);
	public double getInitialFacing();
	public void setInitialFacing(double initialFacing);
	//Result not restricted!!!
	public double getFacingRelative();
	public double getFacingAbsolute();
}
