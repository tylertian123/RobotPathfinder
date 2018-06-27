package robot.pathfinder.core;

import robot.pathfinder.core.path.PathType;

public class TrajectoryParams implements Cloneable {
	
	public Waypoint[] waypoints = null;
	public double alpha = Double.NaN;
	public int segmentCount = 1000;
	public double roundingLimit = 1.0e-6;
	
	public boolean isTank = false;
	
	public PathType pathType = PathType.BEZIER;
	
	@Override
	public TrajectoryParams clone() {
		TrajectoryParams tp = new TrajectoryParams();
		tp.waypoints = this.waypoints;
		tp.alpha = this.alpha;
		tp.segmentCount = this.segmentCount;
		tp.roundingLimit = this.roundingLimit;
		
		tp.isTank = this.isTank;
		
		tp.pathType = this.pathType;
		return tp;
	}
	
	public TrajectoryParams() {
	}
}
