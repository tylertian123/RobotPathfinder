package robot.pathfinder.core.path;

import java.util.ArrayList;

import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.splinesegment.SplineSegment;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.util.Pair;

/**
 * An abstract class that represents a path for the robot to follow. Paths are different from trajectories;
 * they only contain information about the locations the robot will pass through. To follow a path, use a
 * trajectory. This class is the superclass of all path classes.
 * @author Tyler Tian
 *
 */
abstract public class Path {
	
	/*
	 * Even though this class provides implementations for all the methods, it is still kept as abstract, 
	 * because different types of paths use different types of spline segments and thus will have different
	 * constructors. This class cannot have a constructor, as SplineSegment is an interface and cannot be
	 * instantiated.
	 */
	
	/**
	 * Constructs a path.
	 * @param type The type of path to construct; for more information, see {@link PathType}
	 * @param waypoints The {@link Waypoint}s this path has to pass through
	 * @param alpha The turn smoothness constant. A lower value will result in a relatively shorter path with sharper turns
	 * <em>at the waypoints</em>, and a higher value will result in a relatively longer path with smoother turns
	 * <em>at the waypoints</em>. However, since the turns are only smoothed near the waypoints, increasing this
	 * value too much can result in unwanted sharp turns between waypoints.
	 * @return The constructed path
	 */
	public static Path constructPath(PathType type, Waypoint[] waypoints, double alpha) {
		switch(type) {
		case BEZIER:
			return new BezierPath(waypoints, alpha);
		case QUINTIC_HERMITE:
			return new QuinticHermitePath(waypoints, alpha);
		case CUBIC_HERMITE:
			return new CubicHermitePath(waypoints, alpha);
		default:
			throw new IllegalArgumentException("Unknown path type");
		}
	}
	
	Waypoint[] waypoints;
	double alpha;
	SplineSegment[] segments;
	
	PathType type;

	// These values are used later to store the result of computePathLength
	double totalLen = Double.NaN;
	// Lookup table for conversion between length and t
	ArrayList<Pair<Double, Double>> s2tLookupTable = null;
	
	// If set to true, the locations of left and right wheels are reversed
	// This is so that paths generated with the mirror methods are still accurate
	boolean drivingBackwards = false;
	
	double baseRadius;
	
	/**
	 * Sets the base plate radius (distance from the center of the robot to the wheels) of the robot following
	 * this path. This value is used to compute the result from {@link #wheelsAt(double)}.
	 * @param b The new base radius
	 */
	public void setBaseRadius(double b) {
		baseRadius = b;
	}
	/**
	 * Retrieves the base radius (distance from the center of the robot to the wheels) of the robot following
	 * this path. This value is used to compute the result from {@link #wheelsAt(double)}.
	 * @return The base radius
	 */
	public double getBaseRadius() {
		return baseRadius;
	}
	
	/**
	 * Returns the position at a specified time in the path.
	 * @param t A real number in the range [0, 1]
	 * @return The position on this path at the specified time
	 */
	public Vec2D at(double t) {
		// Spline segments take in t values from 0 to 1, so extra conversions are needed
		// If t is more than or equal to 1, just return the end of the last segment
		if(t >= 1) {
			return segments[segments.length - 1].at(1);
		}
		// Otherwise, first multiply by the number of segments
		t *= segments.length;
		// Now the number rounded down is the index of the path, and the number mod 1 is the local t for the segment
		return segments[(int) Math.floor(t)].at(t % 1.0);
	}
	/**
	 * Returns the derivative at a specified time in the path.
	 * @param t A real number in the range [0, 1]
	 * @return The derivative of the path at the specified time
	 */
	public Vec2D derivAt(double t) {
		// For explanations see at()
		if(t >= 1) {
			return segments[segments.length - 1].derivAt(1);
		}
		
		t *= segments.length;
		return segments[(int) Math.floor(t)].derivAt(t % 1.0);
	}
	/**
	 * Returns the second derivative at a specified time in the path.
	 * @param t A real number in the range [0, 1]
	 * @return The second derivative of the path at the specified time
	 */
	public Vec2D secondDerivAt(double t) {
		// For explanations see at()
		if(t >= 1) {
			return segments[segments.length - 1].secondDerivAt(1);
		}
		
		t *= segments.length;
		return segments[(int) Math.floor(t)].secondDerivAt(t % 1.0);
	}
	
	/**
	 * Returns the position of the wheels at a specified time in the path.
	 * @param t A real number in the range [0, 1]
	 * @return The position of the wheels at the specified time; the first element is the left wheel, and the second
	 * element is the right wheel
	 */
	public Vec2D[] wheelsAt(double t) {
		// First get the position of the center and the derivative
		Vec2D position = at(t);
		Vec2D derivative = derivAt(t);
		// Heading can be calculated from the derivative
		double heading = Math.atan2(derivative.getY(), derivative.getX());
		// Store these for multiple use
		double sinHeading = Math.sin(heading);
		double cosHeading = Math.cos(heading);
		/*
		 * sin(x + pi/2) = cos(x)
		 * cos(x + pi/2) = -sin(x)
		 * sin(x - pi/2) = -cos(x)
		 * cos(x - pi/2) = sin(x)
		 */
		Vec2D left = new Vec2D(position.getX() - (!drivingBackwards ? baseRadius * sinHeading : -baseRadius * sinHeading),
				position.getY() + (!drivingBackwards ? baseRadius * cosHeading : -baseRadius * cosHeading));
		Vec2D right = new Vec2D(position.getX() + (!drivingBackwards ? baseRadius * sinHeading : -baseRadius * sinHeading),
				position.getY() - (!drivingBackwards ? baseRadius * cosHeading : -baseRadius * cosHeading));
		
		return new Vec2D[] { left, right };
	}
	
	/**
	 * Prepares for length-to-time conversion by creating a lookup table and computing the path length by numerical integration.
	 * Same as as calling {@link #computePathLength(int)}. This method (or {@link #computePathLength(int)} 
	 * must be called prior to calling {@link #s2T(double)}, or a {@link NullPointerException} will be thrown.
	 * @param points The number of samples to take along the path; more samples lead to more accurate results,
	 * but take more time
	 * @see #computePathLength(int)
	 */
	public void prepareS2T(int points) {
		computePathLength(points);
	}
	/**
	 * Computes the path length by numerical integration and creates the lookup table for length-to-time conversion.
	 * Same as as calling {@link #prepareS2T(int)}. This method (or {@link #prepareS2T(int)} must be called
	 * prior to calling {@link #getPathLength()}, or {@code NaN} will be returned.
	 * @param points The number of samples to take along the path; more samples lead to more accurate results,
	 * but take more time
	 * @return The calculated path length
	 * @see #prepareS2T(int)
	 */
	public double computePathLength(int points) {
		double dt = 1.0 / (points - 1);
		
		Vec2D last = at(0);
		totalLen = 0;
		s2tLookupTable = new ArrayList<>(points);
		s2tLookupTable.add(new Pair<>(0.0, 0.0));
		for(int i = 1; i < points; i ++) {
			// Numerically integrate the path length
			Vec2D current = at(i * dt);
			totalLen += last.distTo(current);
			// Add the path length-time pair to the lookup table
			s2tLookupTable.add(new Pair<>(totalLen, i * dt));
			last = current;
		}
		return totalLen;
	}
	/**
	 * Retrieves the calculated path length. The path length is calculated by means of numerical integration
	 * in the {@link #computePathLength(int)} and {@link #prepareS2T(int)} methods, and <em>must be called prior 
	 * to calling this method</em>. If {@link #computePathLength(int)} or {@link #prepareS2T(int)} was not called,
	 * this method will return {@code NaN}.
	 * @return The previously computed path length
	 */
	public double getPathLength() {
		return totalLen;
	}
	/**
	 * Converts a fractional path length to time. For example, {@code s2T(0.25)} would return the t value upon
	 * which a quarter of the path length was traveled. {@link #prepareS2T(int)} or {@link #computePathLength(int)}
	 * <em>must be called</em> prior to calling this method, or a {@link NullPointerException} will be thrown.
	 * @param s The fraction representing the path length; a real value in the range [0, 1]
	 * @return The time value upon which the specified fraction of the path length has been travelled
	 */
	public double s2T(double s) {
		/*
		 * This method does binary search to find a time value in the lookup table that has the same s value,
		 * or it finds the nearest 2 entries in the table, and linearly interpolates their t value.
		 */
		if(s2tLookupTable == null) {
			throw new NullPointerException("prepareS2T() or computePathLength() must be called before s2T()");
		}
		// Multiply by total path length to find the actual path length (not a fraction)
		double dist = s * totalLen;
		
		int start = 0;
		int end = s2tLookupTable.size() - 1;
		int mid;
		
		// Return 1 if the distance is greater than the maximum distance
		if(dist > s2tLookupTable.get(s2tLookupTable.size() - 1).getElem1()) {
			return 1;
		}
		while(true) {
			mid = (start + end) / 2;
			double midDist = s2tLookupTable.get(mid).getElem1();
			
			// Check if we have a match
			if(midDist == dist) {
				return s2tLookupTable.get(mid).getElem2();
			}
			// Or, check if we are at the end of the lookup table
			if(mid == s2tLookupTable.size() - 1) {
				return 1;
			}
			
			// Check if our distance is in between two distances
			double nextDist = s2tLookupTable.get(mid + 1).getElem1();
			if(midDist <= dist && dist <= nextDist) {
				// Lerp them to get an approximation
				double f = (dist - midDist) / (nextDist - midDist);
				return MathUtils.lerp(s2tLookupTable.get(mid).getElem2(), 
						s2tLookupTable.get(mid + 1).getElem2(), f);
			}
			// Check if we are at the first element after checking if we are between two elements
			// Otherwise, in the case of a value between the first and second, the lerp will not happen
			if(mid == 0) {
				return 0;
			}
			// If not found continue the search
			if(midDist < dist) {
				start = mid;
			}
			else if(midDist > dist) {
				end = mid;
			}
		}
	}
	
	/**
	 * Retrieves the waypoints this path passes through.
	 * @return The path's waypoints
	 */
	public Waypoint[] getWaypoints() {
		return waypoints;
	}
	/**
	 * Retrieves the path smoothness constant of the path. A lower value will result in a relatively shorter path with sharper turns
	 * <em>at the waypoints</em>, and a higher value will result in a relatively longer path with smoother turns
	 * <em>at the waypoints</em>. However, since the turns are only smoothed near the waypoints, increasing this
	 * value too much can result in unwanted sharp turns between waypoints.
	 * @return The path smoothness constant
	 */
	public double getAlpha() {
		return alpha;
	}
	/**
	 * Retrieves the specific type this generic path is.
	 * @return The type of the path
	 */
	public PathType getType() {
		return type;
	}
	
	/**
	 * Sets whether the robot that drives this path is driving backwards or not.
	 * If this is set to true, the locations of the left and right wheels will be reversed.
	 * @param drivingBackwards Whether this path should be driven backwards
	 */
	public void setDrivingBackwards(boolean drivingBackwards) {
		this.drivingBackwards = drivingBackwards;
	}
	
	/**
	 * Returns a modified path, in which every left turn becomes a right turn. Note that is operation is not the
	 * same as reflecting across the Y axis, unless the first waypoint has a heading of pi/2.
	 * @return The mirrored path
	 */
	public Path mirrorLeftRight() {
		// To make it so that left and right turns switch, we have to reflect every waypoint across a line
		// This line has the angle of the first waypoint, so calculate its vector using trig functions
		Vec2D refLine = new Vec2D(Math.cos(waypoints[0].getHeading()), Math.sin(waypoints[0].getHeading()));
		Waypoint[] newWaypoints = new Waypoint[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			// Mirror every waypoint and angle across the reference line
			newWaypoints[i] = new Waypoint(waypoints[i].asVector().reflect(refLine), MathUtils.mirrorAngle(waypoints[i].getHeading(), waypoints[0].getHeading()));
		}
		// Make and return new path
		Path path = constructPath(type, newWaypoints, alpha);
		path.setBaseRadius(baseRadius);
		
		return path;
	}
	/**
	 * Returns a modified path, in which every forward movement becomes a backward movement. Note that this 
	 * operation is not the same as reflecting across the X axis, unless the first waypoint has a heading of
	 * pi/2.
	 * @return The mirrored path
	 */
	public Path mirrorFrontBack() {

		// Like in mirrorLeftRight, we have to first find our reference line
		// That line has angle exactly 90 degrees from the heading of the first waypoint
		// Utilize the property that cos(a + pi/2) = -sin(a) and sin(a + pi/2) = cos(a)
		Vec2D refLine = new Vec2D(-Math.sin(waypoints[0].getHeading()), Math.cos(waypoints[0].getHeading()));
		Waypoint[] newWaypoints = new Waypoint[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			// Reflect everything across the line that has angle 90 degrees more than the first waypoint's heading
			newWaypoints[i] = new Waypoint(waypoints[i].asVector().reflect(refLine), MathUtils.mirrorAngle(waypoints[i].getHeading(), waypoints[0].getHeading() + Math.PI / 2));
		}
		// Construct new path
		Path path = constructPath(type, newWaypoints, alpha);
		path.setBaseRadius(baseRadius);
		// Since the new path is to be driven backwards, set this to true so the wheels' positions will be correct
		path.setDrivingBackwards(true);
		
		return path;
	}
	/**
	 * Returns a path that when driven, will retrace the steps of this path exactly.
	 * @return The retraced path
	 */
	public Path retrace() {
		Waypoint[] newWaypoints = new Waypoint[waypoints.length];
		for(int i = 0; i < waypoints.length; i ++) {
			// New path is just the same as the old path, but with the order of the waypoints reversed,
			// and headings changed. The headings are always 180 degrees apart
			newWaypoints[waypoints.length - 1 - i] = new Waypoint(waypoints[i].getX(), waypoints[i].getY(), (waypoints[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		Path path = constructPath(type, newWaypoints, alpha);
		path.setBaseRadius(baseRadius);
		// Since new path is to be driven backwards, set this to true
		path.setDrivingBackwards(true);
		return path;
	}
}
