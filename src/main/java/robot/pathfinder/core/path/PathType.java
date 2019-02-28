package robot.pathfinder.core.path;

/**
 * An enum of different path types. Differently typed paths use different types of internal spline segments.
 * See the JavaDoc for the values for more information.
 * @author Tyler Tian
 *
 */
public enum PathType {
	/**
	 * The path spline consists of segments of B&#xE9;zier curves. Like {@link CUBIC_HERMITE}, this kind of path
	 * is faster to generate because of their simplicity. However, as B&#xE9;zier spline segments cannot have their
	 * second derivatives constrained, these paths may have small jumps in acceleration where two segments meet.
	 */
	BEZIER,
	/**
	 * The path spline consists of segments of quintic (fifth order) hermite polynomials. Trajectories using these
	 * paths are relatively slower to generate because of their complexity, but realistically this will make a
	 * very small difference. These paths are smoother than other types, and thus it is the recommended type for
	 * trajectory generation (as well as the default for {@link robot.pathfinder.core.TrajectoryParams TrajectoryParams}.
	 */
	QUINTIC_HERMITE,
	/**
	 * The path spline consists of segments of cubic hermite polynomials. Like {@link BEZIER}, this kind of path
	 * is faster to generate because of their simplicity. However, as cubic hermite spline segments cannot have their
	 * second derivatives constrained, these paths may have small jumps in acceleration where two segments meet.
	 */
	CUBIC_HERMITE;
	
	/**
	 * Retrieves the specific class of a path generated with this path type.
	 * <ul>
	 * <li>For {@link #BEZIER}, it is {@link BezierPath}{@code .class};</li>
	 * <li>For {@link #QUINTIC_HERMITE}, it is {@link QuinticHermitePath}{@code .class};</li>
	 * <li>For {@link #CUBIC_HERMITE}, it is {@link CubicHermitePath}{@code .class};</li>
	 * </ul>
	 * @return The specific class of a path generated with this type
	 */
	public Class<?> getPathClass() {
		switch(this) {
		case BEZIER:
			return BezierPath.class;
		case CUBIC_HERMITE:
			return CubicHermitePath.class;
		case QUINTIC_HERMITE:
			return QuinticHermitePath.class;
		default: return null;
		}
	}

	private static final int PT_BEZIER = 1;
    private static final int PT_CUBIC_HERMITE = 2;
    private static final int PT_QUINTIC_HERMITE = 3;

	public int getJNIID() {
		switch(this) {
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