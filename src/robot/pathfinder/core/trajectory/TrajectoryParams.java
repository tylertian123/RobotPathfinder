package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.Waypoint;

public class TrajectoryParams {
	
	public Waypoint[] waypoints = null;
	public double alpha = Double.NaN;
	public int segmentCount = 1000;
	public double roundingLimit = 1.0e-6;
	public boolean isTank = false;
	
	public TrajectoryParams() {
	}
}
