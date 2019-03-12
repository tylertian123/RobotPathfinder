package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.JNIObject;
import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.path.JNIPath;

/**
 * A class that represents a trajectory (motion profile).
 * <p>
 * This is the interface implemented by all trajectory classes. A trajectory not only defines the path the 
 * robot will go through, it also provides information about the velocity, acceleration and direction at
 * every point in time. Using this information, a robot can implement a feedback loop to follow this trajectory.
 * </p> 
 * <h2>Technical Details</h2>
 * <p>
 * Trajectories are generated using numerical integration. This means that it is impossible to have a completely
 * accurate trajectory. However, with enough segments, the error is easily negligible. Trajectories are generated
 * with an algorithm based on the one shown by Team 254 (The Cheesy Poofs) in their video on motion profiling.
 * </p>
 * @author Tyler Tian
 *
 */
public abstract class JNITrajectory extends JNIObject {

	RobotSpecs specs;
	JNITrajectoryParams params;

	@Override
    public void free() {
        super.free();
        clearPathCache();
        clearMomentsCache();
    }
    @Override
    public void finalize() {
        super.finalize();
        clearPathCache();
        clearMomentsCache();
    }
    @Override
    public void close() {
        super.close();
        clearPathCache();
        clearMomentsCache();
    }

    // Native
    abstract protected int _getMomentCount();
	abstract protected void _getMoments();
    abstract public Moment[] getMoments();
    abstract public void clearMomentsCache();
	
	// Native
	abstract protected Moment _get(double t);
	public Moment get(double t) {
		if(Double.isNaN(t) || !Double.isFinite(t)) {
            throw new IllegalArgumentException("Time must be finite and not NaN");
        }
        return _get(t);
	}
	
	// Native
	abstract protected long _getPath();
	protected JNIPath pathCache;
    public JNIPath getPath() {
        if(pathCache == null) {
            pathCache = new JNIPath(params.waypoints, params.alpha, params.pathType, _getPath());
            pathCache.setBaseRadius(specs.getBaseWidth() / 2);
            pathCache._updateWaypoints();
        }
        return pathCache;
    }
    public void clearPathCache() {
        pathCache = null;
    }
	
	public RobotSpecs getRobotSpecs() {
		return specs;
	}
	public JNITrajectoryParams getGenerationParams() {
		return params;
	}

	// Native
	public abstract double totalTime();
	
	abstract public JNITrajectory mirrorLeftRight();
	abstract public JNITrajectory mirrorFrontBack();
	abstract public JNITrajectory retrace();
}
