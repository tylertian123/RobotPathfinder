package robot.pathfinder.core;

/**
 * A class representing the specifications for a robot. <br>
 * <br>
 * This class stores information about the robot's max velocity, acceleration and base width,
 * and can be used in the construction of trajectory objects as an alternative to typing up 
 * the specifications each time.
 * @author Tyler Tian
 *
 */
public class RobotSpecs {
	
	double baseWidth;
	double maxVelocity, maxAcceleration;
	
	/**
	 * Constructs a new robot specification object with the specified values.
	 * @param maxVelocity The absolute value of the max velocity of the robot
	 * @param maxAcceleration The absolute value of the max acceleration of the robot
	 * @param baseWidth The width of the base plate of the robot (distance between wheels on different sides)
	 */
	public RobotSpecs(double maxVelocity, double maxAcceleration, double maxJerk, double baseWidth) {
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
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
