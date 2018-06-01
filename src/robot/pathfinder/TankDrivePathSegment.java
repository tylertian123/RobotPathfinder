package robot.pathfinder;

/**
 * Represents a segment of a path for a tank drive robot. This class is intended for use internally by trajectory
 * generators only.
 * @author Tyler Tian
 *
 */
public class TankDrivePathSegment {
	
	double start, end;
	double leftMaxVelocity, rightMaxVelocity;
	
	/**
	 * Constructs a new segment with the specified values.
	 * @param start - The start of the segment
	 * @param end - The end of the segment
	 */
	public TankDrivePathSegment(double start, double end) {
		this.start = start;
		this.end = end;
	}
	/**
	 * Constructs a new segment with the specified values.
	 * @param start - The start of the segment
	 * @param end - The end of the segment
	 * @param leftMaxVelocity - The max velocity of the left side in the segment
	 * @param rightMaxVelocity - The max velocity of the right side in the segment
	 */
	public TankDrivePathSegment(double start, double end, double leftMaxVelocity, double rightMaxVelocity) {
		this(start, end);
		this.leftMaxVelocity = leftMaxVelocity;
		this.rightMaxVelocity = rightMaxVelocity;
	}
	
	/**
	 * Sets the maximum velocities for both sides in this segment.
	 * @param leftMaxVelocity - The max velocity of the left side in the segment
	 * @param rightMaxVelocity - The max velocity of the right side in the segment
	 */
	public void setMaxVelocities(double leftMaxVelocity, double rightMaxVelocity) {
		this.leftMaxVelocity = leftMaxVelocity;
		this.rightMaxVelocity = rightMaxVelocity;
	}
	/**
	 * Retrieves the max velocity for the left side in this segment.
	 * @return The max velocity of the left side in the segment
	 */
	public double getLeftMaxVelocity() {
		return leftMaxVelocity;
	}
	/**
	 * Retrieves the max velocity for the right side in this segment.
	 * @return The max velocity of the right side in the segment
	 */
	public double getRightMaxVelocity() {
		return rightMaxVelocity;
	}
	/**
	 * Retrieves the start of the segment
	 * @return - The starting point of the segment
	 */
	public double getStart() {
		return start;
	}
	/**
	 * Retrieves the end of the segment
	 * @return - The ending point of the segment
	 */
	public double getEnd() {
		return end;
	}
}
