package robot.pathfinder.core;

public class TrajectoryParams {
	
	public Waypoint[] waypoints = null;
	public double alpha = Double.NaN;
	public int segmentCount = 1000;
	public boolean isTank = false;
	
	public TrajectoryParams() {
	}
}
