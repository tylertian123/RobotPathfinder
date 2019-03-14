package com.arctos6135.robotpathfinder.core.path;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.lifecycle.GlobalLifeCycleManager;
import com.arctos6135.robotpathfinder.core.lifecycle.JNIObject;
import com.arctos6135.robotpathfinder.math.Vec2D;
import com.arctos6135.robotpathfinder.util.Pair;

/**
 * A class that represents a path for the robot to follow. Paths are different
 * from trajectories; they only contain information about the locations the
 * robot will pass through. To follow a path, use a trajectory. This class is
 * the superclass of all path classes.
 * <h2>Memory Management</h2>
 * <p>
 * Each Path has a Java part (the object itself) and a part that resides in
 * native code (stored as a pointer casted into a {@code long}). Because these
 * objects contain handles to native resources that cannot be automatically
 * released by the JVM, the {@link #free()} or {@link #close()} method must be
 * called to free the native resource when the object is no longer needed.
 * </p>
 * <p>
 * <em> Note: Almost all RobotPathfinder JNI classes have some kind of reference
 * counting. However, this reference count is only increased when an object is
 * created or copied by a method, and not when the reference is copied through
 * assignment. <br>
 * For example:
 * 
 * <pre>
 * Path p0 = someTrajectory.getPath();
 * Path p1 = someTrajectory.getPath();
 * p0.free();
 * p1.at(0); // This is valid, because the native resource was never freed due to
 *           // reference counting
 * </pre>
 * 
 * But:
 * 
 * <pre>
 * Path p0 = someTrajectory.getPath();
 * Path p1 = p0;
 * p0.free();
 * p1.at(0); // This will throw an IllegalStateException, since the native resource has
 *           // already been freed
 * </pre>
 * 
 * </em>
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class Path extends JNIObject {

    static {
        GlobalLibraryLoader.load();
        GlobalLifeCycleManager.initialize();
    }

    private native void _construct(Waypoint[] waypoints, double alpha, int type);

    protected PathType type;
    protected Waypoint[] waypoints;
    protected double alpha;

    /**
     * Creates a new {@link Path} with the specified waypoints, alpha, and type.
     * <p>
     * Alpha is the turn smoothness constant. A lower value will result in a
     * relatively shorter path with sharper turns <em>at the waypoints</em>, and a
     * higher value will result in a relatively longer path with smoother turns
     * <em>at the waypoints</em>. However, since the turns are only smoothed near
     * the waypoints, increasing this value too much can result in unwanted sharp
     * turns between waypoints.
     * </p>
     * 
     * @param waypoints The waypoints this path must pass through
     * @param alpha     The turn smoothness constant
     * @param type      The type of the path
     */
    public Path(Waypoint[] waypoints, double alpha, PathType type) {
        this.type = type;
        this.waypoints = waypoints;
        this.alpha = alpha;
        if (waypoints.length < 2) {
            throw new IllegalArgumentException("Not enough waypoints");
        }

        _construct(waypoints, alpha, type.getJNIID());
        GlobalLifeCycleManager.register(this);
    }

    /**
     * Creates a new {@link Path} directly from a native pointer.
     * <p>
     * <b><em>This constructor is intended for internal use only. Use at your own
     * risk.</em></b>
     * </p>
     * 
     * @param waypoints The waypoints of the path
     * @param alpha     The alpha of the path
     * @param type      The type of the path
     * @param ptr       A pointer to the native object
     */
    public Path(Waypoint[] waypoints, double alpha, PathType type, long ptr) {
        this.waypoints = waypoints;
        this.alpha = alpha;
        this.type = type;
        _nativePtr = ptr;
        GlobalLifeCycleManager.register(this);
    }

    protected native void _destroy();

    private native void _setBaseRadius(double radius);

    private native void _setBackwards(boolean backwards);

    protected double radius;
    protected boolean backwards = false;

    /**
     * Sets the base plate radius (distance from the center of the robot to the
     * wheels) of the robot following this path. This value is used to compute the
     * result from {@link #wheelsAt(double)}.
     * 
     * @param radius The new base radius
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public void setBaseRadius(double radius) {
        this.radius = radius;
        _setBaseRadius(radius);
    }

    /**
     * Sets whether the robot that drives this path is driving backwards or not. If
     * this is set to true, the locations of the left and right wheels will be
     * reversed.
     * 
     * @param backwards Whether this path should be driven backwards
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public void setDrivingBackwards(boolean backwards) {
        this.backwards = backwards;
        _setBackwards(backwards);
    }

    /**
     * Retrieves the base radius (distance from the center of the robot to the
     * wheels) of the robot following this path. This value is used to compute the
     * result from {@link #wheelsAt(double)}.
     * 
     * @return The base radius
     */
    public double getBaseRadius() {
        return radius;
    }

    /**
     * Gets whether the robot that drives this path is driving backwards or not. If
     * this is set to true, the locations of the left and right wheels will be
     * reversed.
     * 
     * @return Whether or not the path is driven backwards
     */
    public boolean getDrivingBackwards() {
        return backwards;
    }

    /**
     * Retrieves the waypoints this path passes through.
     * 
     * @return The path's waypoints
     */
    public Waypoint[] getWaypoints() {
        return waypoints;
    }

    /**
     * Retrieves the alpha of the path.
     * <p>
     * Alpha is the turn smoothness constant. A lower value will result in a
     * relatively shorter path with sharper turns <em>at the waypoints</em>, and a
     * higher value will result in a relatively longer path with smoother turns
     * <em>at the waypoints</em>. However, since the turns are only smoothed near
     * the waypoints, increasing this value too much can result in unwanted sharp
     * turns between waypoints.
     * </p>
     * 
     * @return The path smoothness constant
     */
    public double getAlpha() {
        return alpha;
    }

    /**
     * Retrieves the position at a specified time in the path.
     * <p>
     * Note that this method does not take into account the lengths of the segments
     * of the path. Rather it divides the total time evenly into equal sized
     * segments for each segment, regardless of their lengths. This means that even
     * though some segments may be longer than others, they still take up the same
     * amount of time.
     * </p>
     * 
     * @param time A real number in the range [0, 1]
     * @return The position on this path at the specified time
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public native Vec2D at(double time);

    /**
     * Retrieves the derivative of the position at a specified time in the path.
     * <p>
     * Note that this method does not take into account the lengths of the segments
     * of the path. Rather it divides the total time evenly into equal sized
     * segments for each segment, regardless of their lengths. This means that even
     * though some segments may be longer than others, they still take up the same
     * amount of time.
     * </p>
     * 
     * @param time A real number in the range [0, 1]
     * @return The derivative on this path at the specified time
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public native Vec2D derivAt(double time);

    /**
     * Retrieves the second derivative of the position at a specified time in the
     * path.
     * <p>
     * Note that this method does not take into account the lengths of the segments
     * of the path. Rather it divides the total time evenly into equal sized
     * segments for each segment, regardless of their lengths. This means that even
     * though some segments may be longer than others, they still take up the same
     * amount of time.
     * </p>
     * 
     * @param time A real number in the range [0, 1]
     * @return The second derivative on this path at the specified time
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public native Vec2D secondDerivAt(double time);

    /**
     * Retrieves the position of the wheels at a specified time in the path. The
     * returned value is a pair of vectors, in which the first vector is the
     * position of the left wheel, and the second vector is the position of the
     * right wheel.
     * <p>
     * Note that this method does not take into account the lengths of the segments
     * of the path. Rather it divides the total time evenly into equal sized
     * segments for each segment, regardless of their lengths. This means that even
     * though some segments may be longer than others, they still take up the same
     * amount of time.
     * </p>
     * 
     * @param time A real number in the range [0, 1]
     * @return A pair of positions (left, right) of the wheels on the path at the
     *         specified time
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public native Pair<Vec2D, Vec2D> wheelsAt(double time);

    private native double _computeLen(int points);

    private native double _s2T(double s);

    private native double _t2S(double t);

    protected double length = Double.NaN;

    /**
     * Computes the length of the path with the given number of points. The length
     * is computed with numerical integration, and a larger number will take longer
     * to compute, but will result in increased accuracy. This method must be called
     * prior to {@link #getLength()}, {@link #s2T(double)} and {@link #t2S(double)}.
     * 
     * @param points The number of points to take along the path for integration
     * @return The total length of the path
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public double computeLen(int points) {
        length = _computeLen(points);
        return length;
    }

    /**
     * Retrieves the length of the path computed with numerical integration.
     * {@link #computeLen(int)} must be called before this method, otherwise an
     * {@link IllegalStateException} will be thrown.
     * 
     * @return The total length of the path
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc), or if
     *                               {@link #computeLen(int)} was never called
     */
    public double getLength() {
        if (length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return length;
    }

    /**
     * Converts a fractional path length to time using a lookup table. For example,
     * {@code s2T(0.25)} would return the t value upon which a quarter of the path
     * length was traveled. {@link #computeLen(int)} <em>must be called</em> prior
     * to calling this method, or a {@link IllegalStateException} will be thrown.
     * 
     * @param s The fraction of the total path length
     * @return The time upon which s has been travelled
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc), or if
     *                               {@link #computeLen(int)} was never called
     */
    public double s2T(double s) {
        if (length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return _s2T(s);
    }

    /**
     * Converts a time in the range [0, 1] to a fractional path length using a
     * lookup table. For example, {@code t2S(0.25)} would return a fraction
     * representing the amount of the total path length travelled at time=0.25.
     * {@link #computeLen(int)} <em>must be called</em> prior to calling this
     * method, or a {@link IllegalStateException} will be thrown.
     * 
     * @param t A time in the range [0, 1]
     * @return The fraction of the total path length travelled at the time specified
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc), or if
     *                               {@link #computeLen(int)} was never called
     */
    public double t2S(double t) {
        if (length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return _t2S(t);
    }

    private native long _mirrorLeftRight();

    private native long _mirrorFrontBack();

    private native long _retrace();

    /**
     * Updates the cached waypoints in this class.
     * <p>
     * <b><em>This method is intended for internal use only. Use at your own
     * risk.</em></b>
     * </p>
     */
    public native void _updateWaypoints();

    /**
     * Constructs a new path, in which every left turn becomes a right turn.
     * 
     * @return The new path
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public Path mirrorLeftRight() {
        Path p = new Path(waypoints, alpha, type, _mirrorLeftRight());
        p.backwards = backwards;
        p.radius = radius;
        p.waypoints = new Waypoint[waypoints.length];
        p._updateWaypoints();
        return p;
    }

    /**
     * Constructs a new path, in which the direction of driving is reversed
     * (i.e. driving forwards is now driving backwards).
     * 
     * @return The new path
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public Path mirrorFrontBack() {
        Path p = new Path(waypoints, alpha, type, _mirrorFrontBack());
        p.backwards = !backwards;
        p.radius = radius;
        p.waypoints = new Waypoint[waypoints.length];
        p._updateWaypoints();
        return p;
    }

    /**
     * Constructs a new path, which, if driven out from the end of this path, will
     * retrace the steps of this path exactly and return to where this path started.
     * 
     * @return The new path
     * @throws IllegalStateException If the native resource has already been freed
     *                               (see class JavaDoc)
     */
    public Path retrace() {
        Path p = new Path(waypoints, alpha, type, _retrace());
        p.backwards = !backwards;
        p.radius = radius;
        p.waypoints = new Waypoint[waypoints.length];
        p._updateWaypoints();
        return p;
    }
}
