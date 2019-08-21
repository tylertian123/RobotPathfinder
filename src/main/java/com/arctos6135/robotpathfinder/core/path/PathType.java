package com.arctos6135.robotpathfinder.core.path;

/**
 * An enum of different path types. Differently typed paths use different types
 * of internal spline segments. See the Javadoc for the values for more
 * information.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public enum PathType {
	/**
	 * The path spline consists of segments of B&#xE9;zier curves. Like
	 * {@link #CUBIC_HERMITE}, this kind of path is faster to generate because of
	 * their simplicity. However, as B&#xE9;zier spline segments cannot have their
	 * second derivatives constrained, these paths may have small jumps in
	 * acceleration where two segments meet.
	 */
	BEZIER,
	/**
	 * The path spline consists of segments of quintic (fifth order) hermite
	 * polynomials. Trajectories using these paths are relatively slower to generate
	 * because of their complexity, but realistically this will make a very small
	 * difference. These paths are smoother than other types, and thus it is the
	 * recommended type for trajectory generation (as well as the default for
	 * {@link com.arctos6135.robotpathfinder.core.TrajectoryParams
	 * TrajectoryParams}.
	 */
	QUINTIC_HERMITE,
	/**
	 * The path spline consists of segments of cubic hermite polynomials. Like
	 * {@link #BEZIER}, this kind of path is faster to generate because of their
	 * simplicity. However, as cubic hermite spline segments cannot have their
	 * second derivatives constrained, these paths may have small jumps in
	 * acceleration where two segments meet.
	 */
	CUBIC_HERMITE;

	private static final int PT_BEZIER = 1;
	private static final int PT_CUBIC_HERMITE = 2;
	private static final int PT_QUINTIC_HERMITE = 3;

	/**
	 * Retrieves the JNI enum value of this {@link PathType}.
	 * <p>
	 * <b><em>This method is intended for internal use only. Use at your own
	 * risk.</em></b>
	 * </p>
	 * 
	 * @return The native enum value for this {@link PathType}
	 */
	public int getJNIID() {
		switch (this) {
		case BEZIER:
			return PT_BEZIER;
		case CUBIC_HERMITE:
			return PT_CUBIC_HERMITE;
		case QUINTIC_HERMITE:
			return PT_QUINTIC_HERMITE;
		default:
			return 0;
		}
	}
}
