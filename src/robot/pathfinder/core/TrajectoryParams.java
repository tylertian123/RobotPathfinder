package robot.pathfinder.core;

public class TrajectoryParams {
	
	public enum PathType {
		BEZIER,
		QUINTIC_HERMITE,
	}
	
	public Waypoint[] waypoints = null;
	public double alpha = Double.NaN;
	public int segmentCount = 1000;
	public double roundingLimit = 1.0e-6;
	
	public boolean isTank = false;
	
	public PathType pathType = PathType.BEZIER;
	
	public TrajectoryParams() {
	}
}
