package com.arctos6135.robotpathfinder.core.trajectory;

/**
 * Signals that an error with trajectory generation has occurred.
 * 
 * @author Tyler Tian
 * @since 3.0.0
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
