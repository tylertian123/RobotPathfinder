package robot.pathfinder.core;

/**
 * Thrown when attempting to modify a {@link Moment} object that has been locked.
 * @author Tyler Tian
 *
 */
public class LockedException extends RuntimeException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 8646894561403039565L;

	public LockedException() {
		super();
	}
	public LockedException(String message) {
		super(message);
	}
}
