package robot.pathfinder.core;

/**
 * Signals that an error with trajectory generation has occurred. Thrown by trajectory generation methods
 * when a trajectory is impossible.
 * @author Tyler Tian
 *
 */
public class TrajectoryGenerationException extends RuntimeException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 4458957792647962346L;
	
	public TrajectoryGenerationException(String message) {
		super(message);
	}
	public TrajectoryGenerationException() {
		super();
	}
}
