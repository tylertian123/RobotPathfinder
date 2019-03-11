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
        momentsCache = null;
        pathCache = null;
    }
    @Override
    public void finalize() {
        super.finalize();
        momentsCache = null;
        pathCache = null;
    }
    @Override
    public void close() {
        super.close();
        momentsCache = null;
        pathCache = null;
    }

	// Native
	abstract protected void _getMoments();
	protected Moment[] momentsCache;
    public Moment[] getMoments() {
        if(momentsCache == null) {
            momentsCache = new BasicMoment[params.segmentCount];
            _getMoments();
        }
        return momentsCache;
    }
    public void clearMomentsCache() {
        momentsCache = null;
    }
	
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
