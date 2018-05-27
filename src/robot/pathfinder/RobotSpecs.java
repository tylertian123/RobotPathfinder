package robot.pathfinder;

/**
 * A class representing the specifications for a robot. <br>
 * <br>
 * This class stores information about the robot's max velocity, acceleration and base width,
 * and can be used in the construction of trajectory objects as an alternative to typing up 
 * the specifications each time.
 * @author Tyler
 *
 */
public class RobotSpecs {
	
	double baseWidth;
	double maxVelocity, maxAcceleration, maxDeceleration;
	
	/**
	 * Constructs a new robot specification object with the specified values. This constructor sets the value of the max
	 * deceleration to be the same as the max acceleration.
	 * @param maxVelocity - The absolute value of the max velocity of the robot
	 * @param maxAcceleration - The absolute value of the max acceleration of the robot
	 * @param baseWidth - The width of the base plate of the robot (distance between wheels on different sides)
	 */
	public RobotSpecs(double maxVelocity, double maxAcceleration, double baseWidth) {
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = this.maxDeceleration = maxAcceleration;
		this.baseWidth = baseWidth;
	}
	/**
	 * Constructs a new robot specification object with the specified values.
	 * @param maxVelocity - The absolute value of the max velocity of the robot
	 * @param maxAcceleration - The absolute value of the max acceleration of the robot
	 * @param maxDeceleration - The absolute value of the max deceleration of the robot
	 * @param baseWidth - The width of the robot's base
	 */
	public RobotSpecs(double maxVelocity, double maxAcceleration, double maxDeceleration, double baseWidth) {
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxDeceleration = maxDeceleration;
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
	 * @param baseWidth - The new base width
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
	 * @param maxVelocity - The new max velocity
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
	 * @param maxAcceleration - The new max acceleration
	 */
	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}
	/**
	 * Retrieves the max deceleration of this robot specifications object. By default this value is the same as the max acceleration.
	 * @return
	 */
	public double getMaxDeceleration() {
		return maxDeceleration;
	}
	/**
	 * Sets the max deceleration of this robot specifications object.
	 * @param maxDeceleration - The max deceleration of the robot
	 */
	public void setMaxDeceleration(double maxDeceleration) {
		this.maxDeceleration = maxDeceleration;
	}
}
