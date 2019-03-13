package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.lifecycle.GlobalLifeCycleManager;

public class BasicTrajectory extends Trajectory {

    static {
        GlobalLibraryLoader.load();
        GlobalLifeCycleManager.initialize();
    }

    private native void _construct(double maxV, double maxA, double baseWidth, boolean isTank, Waypoint[] waypoints, 
            double alpha, int sampleCount, int type);

    public BasicTrajectory(RobotSpecs specs, TrajectoryParams params) {
        if(Double.isNaN(specs.getMaxVelocity())) {
            throw new IllegalArgumentException("Max velocity cannot be NaN");
        }
        if(Double.isNaN(specs.getMaxAcceleration())) {
            throw new IllegalArgumentException("Max acceleration cannot be NaN");
        }
        if(params.waypoints == null) {
            throw new IllegalArgumentException("Waypoints not set");
        }
        if(Double.isNaN(params.alpha)) {
            throw new IllegalArgumentException("Alpha cannot be NaN");
        }
        if(params.sampleCount < 1) {
            throw new IllegalArgumentException("Segment count must be greater than zero");
        }

        this.specs = specs;
        this.params = params;

        _construct(specs.getMaxVelocity(), specs.getMaxAcceleration(), specs.getBaseWidth(), false, params.waypoints, 
                params.alpha, params.sampleCount, params.pathType.getJNIID());
        GlobalLifeCycleManager.register(this);
    }
    public BasicTrajectory(RobotSpecs specs, TrajectoryParams params, long ptr) {
        this.specs = specs;
        this.params = params;
        _nativePtr = ptr;
        GlobalLifeCycleManager.register(this);
    }

    @Override
    protected native void _destroy();

    @Override
    protected native int _getMomentCount();
    @Override
    protected native void _getMoments();
    protected BasicMoment[] momentsCache;
    @Override
    public BasicMoment[] getMoments() {
        if(momentsCache == null) {
            momentsCache = new BasicMoment[_getMomentCount()];
            _getMoments();
        }
        return momentsCache;
    }
    @Override
    public void clearMomentsCache() {
        momentsCache = null;
    }

    @Override
    protected native BasicMoment _get(double t);
    @Override
    public BasicMoment get(double t) {
        return (BasicMoment) super.get(t);
    }

    @Override
    protected native long _getPath();

    @Override
    public native double totalTime();

    private native long _mirrorLeftRight();
    @Override
    public BasicTrajectory mirrorLeftRight() {
        return new BasicTrajectory(specs, params, _mirrorLeftRight());
    }

    private native long _mirrorFrontBack();
    @Override
    public BasicTrajectory mirrorFrontBack() {
        return new BasicTrajectory(specs, params, _mirrorFrontBack());
    }

    private native long _retrace();
    @Override
    public BasicTrajectory retrace() {
        return new BasicTrajectory(specs, params, _retrace());
    }

}
