package com.arctos6135.robotpathfinder.core;

import java.util.Arrays;
import java.util.Objects;

import com.arctos6135.robotpathfinder.core.path.PathType;

/**
 * A collection of parameters for trajectory generation. Used in the
 * construction of trajectories.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class TrajectoryParams implements Cloneable {

	/**
	 * The {@link Waypoint}s that this trajectory's path has to pass through.
	 * Default value is {@code null}.
	 */
	public Waypoint[] waypoints = null;
	/**
	 * The turn smoothness constant.
	 * <p>
	 * A lower value will result in a relatively shorter path with sharper turns
	 * <em>at the waypoints</em>, and a higher value will result in a relatively
	 * longer path with smoother turns <em>at the waypoints</em>. However, since the
	 * turns are only smoothed near the waypoints, increasing this value too much
	 * can result in unwanted sharp turns between waypoints. Default value is
	 * {@code NaN}. This value has no unit.
	 * </p>
	 * <p>
	 * Internally, this value is used to compute the derivatives when fitting
	 * hermite splines. The sine and cosine values of the heading of the waypoint
	 * are multiplied by this value to get the derivative.
	 * </p>
	 */
	public double alpha = Double.NaN;
	/**
	 * The number of samples to make along the path when constructing trajectories.
	 * Default value is {@code 1000}.
	 * <p>
	 * Trajectories are constructed using numerical integration. Thus, a higher
	 * sample count leads to a more accurate trajectory, but also takes longer to
	 * generate.
	 * </p>
	 */
	public int sampleCount = 1000;
	/**
	 * The type of path to be used by the trajectory. For more information, see
	 * {@link PathType}. Default value is {@link PathType#QUINTIC_HERMITE}.
	 */
	public PathType pathType = PathType.QUINTIC_HERMITE;

	/**
	 * Creates an identical copy of this {@link TrajectoryParams}.
	 */
	@Override
	public TrajectoryParams clone() {
		TrajectoryParams tp = new TrajectoryParams();
		tp.waypoints = this.waypoints;
		tp.alpha = this.alpha;
		tp.sampleCount = this.sampleCount;
		tp.pathType = this.pathType;
		return tp;
	}

	@Override
	public boolean equals(Object o) {
		if (o == this)
			return true;
		if (!(o instanceof TrajectoryParams)) {
			return false;
		}
		TrajectoryParams t = (TrajectoryParams) o;
		return Arrays.equals(waypoints, t.waypoints) && alpha == t.alpha && sampleCount == t.sampleCount
				&& pathType == t.pathType;
	}

	@Override
	public int hashCode() {
		return Objects.hash(waypoints, alpha, sampleCount, pathType);
	}

	@Override
	public String toString() {
		return "{" + " waypoints='" + waypoints + "'" + ", alpha='" + alpha + "'" + ", sampleCount='" + sampleCount
				+ "'" + ", pathType='" + pathType + "'" + "}";
	}

	/**
	 * Constructs a new {@link TrajectoryParams} object with all fields set to their
	 * default values.
	 */
	public TrajectoryParams() {
	}
}
