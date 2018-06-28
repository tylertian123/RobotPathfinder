package robot.pathfinder.core.path;

public enum PathType {
	BEZIER,
	QUINTIC_HERMITE,
	CUBIC_HERMITE;
	
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
}