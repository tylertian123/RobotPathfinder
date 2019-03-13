package com.arctos6135.robotpathfinder.core;

import com.arctos6135.robotpathfinder.core.path.PathType;

/**
 * A collection of parameters for trajectory generation. Used in the construction of {@link com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory BasicTrajectory}.
 * @author Tyler Tian
 *
 */
public class TrajectoryParams implements Cloneable {
	
	/**
	 * The {@link Waypoint}s that this trajectory's path has to pass through. Default value is {@code null}.
	 * <p>
	 * Note that it does not matter what specific unit is used for distance; however, the unit must match with 
	 * the units in the {@link RobotSpecs} object used to construct the trajectory. For example, if the unit
	 * for max velocity in the {@link RobotSpecs} object was m/s, the unit used for distance must be m.
	 * </p>
	 */
	public Waypoint[] waypoints = null;
	/**
	 * The turn smoothness constant. A lower value will result in a relatively shorter path with sharper turns
	 * <em>at the waypoints</em>, and a higher value will result in a relatively longer path with smoother turns
	 * <em>at the waypoints</em>. However, since the turns are only smoothed near the waypoints, increasing this
	 * value too much can result in unwanted sharp turns between waypoints. Default value is {@code NaN}.
	 * <p>
	 * Internally, this value is used to compute the derivatives when fitting hermite splines. The sine and cosine
	 * values of the heading of the waypoint are multiplied by this value to get the derivative.
	 * </p>
	 */
	public double alpha = Double.NaN;
	
	public int sampleCount = 1000;
	/**
	 * The type of path to be used by the trajectory. For more information, see {@link PathType}. Default value is
	 * {@link PathType#QUINTIC_HERMITE}.
	 */
	public PathType pathType = PathType.QUINTIC_HERMITE;
	
	@Override
	public TrajectoryParams clone() {
		TrajectoryParams tp = new TrajectoryParams();
		tp.waypoints = this.waypoints;
		tp.alpha = this.alpha;
		tp.sampleCount = this.sampleCount;
		
		tp.pathType = this.pathType;
		return tp;
	}
	
	/**
	 * Constructs a new {@link TrajectoryParams} object with all fields set to their default values.
	 */
	public TrajectoryParams() {
	}
}
