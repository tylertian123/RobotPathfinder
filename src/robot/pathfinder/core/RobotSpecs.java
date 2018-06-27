package robot.pathfinder.core;

/**
 * A class representing the specifications for a robot. Robot specifications consist of a maximum velocity and
 * acceleration, and in the case of tank drive robots, the width of the base plate. Used in the construction of
 * {@link BasicTrajectory}.
 * @author Tyler Tian
 *
 */
public class RobotSpecs {
	
	double baseWidth = Double.NaN;
	double maxVelocity, maxAcceleration;
	
	/**
	 * Constructs a new robot specification object with the specified values.
	 * The value for base plate width is set to {@code NaN}.
	 * @param maxVelocity The absolute value of the max velocity of the robot
	 * @param maxAcceleration The absolute value of the max acceleration of the robot
	 */
	public RobotSpecs(double maxVelocity, double maxAcceleration) {
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
	}
	/**
	 * Constructs a new robot specification object with the specified values.
	 * @param maxVelocity The absolute value of the max velocity of the robot
	 * @param maxAcceleration The absolute value of the max acceleration of the robot
	 * @param baseWidth The width of the base plate of the robot (distance from wheels on one side to wheels on
	 * the other side)
	 */
	public RobotSpecs(double maxVelocity, double maxAcceleration, double baseWidth) {
		this(maxVelocity, maxAcceleration);
		this.baseWidth = baseWidth;
	}
	
	
	/**
	 * Retrieves the base width of this robot specifications object.
	 * @return The width of the robot's base plate
	 */
	public double getBaseWidth() {
		return baseWidth;
	}
	/**
	 * Sets the base width of this robot specifications object.
	 * @param baseWidth The new base width
	 */
	public void setBaseWidth(double baseWidth) {
		this.baseWidth = baseWidth;
	}
	/**
	 * Retrieves the max velocity of this robot specifications object.
	 * @return The max velocity of the robot
	 */
	public double getMaxVelocity() {
		return maxVelocity;
	}
	/**
	 * Sets the max velocity of this robot specifications object.
	 * @param maxVelocity The new max velocity
	 */
	public void setMaxVelocity(double maxVelocity) {
		this.maxVelocity = maxVelocity;
	}
	/**
	 * Retrieves the max acceleration of this robot specifications object.
	 * @return The max acceleration of the robot
	 */
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	/**
	 * Sets the max acceleration of this robot specifications object.
	 * @param maxAcceleration The new max acceleration
	 */
	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}
}
