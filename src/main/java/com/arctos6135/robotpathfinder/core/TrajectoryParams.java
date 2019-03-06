package com.arctos6135.robotpathfinder.core;

import com.arctos6135.robotpathfinder.core.path.PathType;

/**
 * A collection of parameters for trajectory generation. Used in the construction of {@link robot.pathfinder.core.trajectory.BasicTrajectory BasicTrajectory}.
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
	/**
	 * The number of segments to split the path into when calculating. Because trajectory generation uses numerical
	 * integration, a higher segment count will result in a better approximation of the theoretical perfect trajectory,
	 * but will increase the generation time. Default value is {@code 1000}.
	 */
	public int segmentCount = 1000;
	/**
	 * The rounding limit for small floating-point numbers when solving equations. During calculations, 
	 * floating-point rounding errors can stack up, resulting in unsolvable equations. Any number smaller in
	 * magnitude than this value will be rounded down to 0. This field can usually be ignored. Default value is
	 * {@code 1.0e-6}.
	 * @deprecated This field is no longer needed in the current implementation. It is only kept for compatibility.
	 */
    @Deprecated
	public double roundingLimit = 1.0e-6;
	/**
	 * Whether the trajectory is to be used as the base for a {@link robot.pathfinder.core.trajectory.TankDriveTrajectory TankDriveTrajectory}. Setting this value to
	 * {@code true} will result in extra processing to prepare for the generation of a {@link robot.pathfinder.core.trajectory.TankDriveTrajectory TankDriveTrajectory}.
	 * If a trajectory is generated with this field set to {@code false}, and passed into the constructor of a 
	 * {@link robot.pathfinder.core.trajectory.TankDriveTrajectory TankDriveTrajectory}, an {@link IllegalArgumentException} will be thrown. Default value is {@code false}.
	 */
	public boolean isTank = false;
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
		tp.segmentCount = this.segmentCount;
		tp.roundingLimit = this.roundingLimit;
		
		tp.isTank = this.isTank;
		
		tp.pathType = this.pathType;
		return tp;
	}
	
	/**
	 * Constructs a new {@link TrajectoryParams} object with all fields set to their default values.
	 */
	public TrajectoryParams() {
	}
}
