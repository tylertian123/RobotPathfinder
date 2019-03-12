package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;
import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;

public class JNITankDriveTrajectory extends JNITrajectory {

    static {
        GlobalLibraryLoader.load();
    }

    private native void _construct(double maxV, double maxA, double baseWidth, boolean isTank, JNIWaypoint[] waypoints, 
            double alpha, int segmentCount, int type);

    public JNITankDriveTrajectory(RobotSpecs specs, JNITrajectoryParams params) {
        if(Double.isNaN(specs.getMaxVelocity())) {
            throw new IllegalArgumentException("Max velocity cannot be NaN");
        }
        if(Double.isNaN(specs.getMaxAcceleration())) {
            throw new IllegalArgumentException("Max acceleration cannot be NaN");
        }
        if(params.isTank && Double.isNaN(specs.getBaseWidth())) {
            throw new IllegalArgumentException("Base width cannot be NaN if trajectory is tank drive");
        }
        if(params.waypoints == null) {
            throw new IllegalArgumentException("Waypoints not set");
        }
        if(Double.isNaN(params.alpha)) {
            throw new IllegalArgumentException("Alpha cannot be NaN");
        }
        if(params.segmentCount < 1) {
            throw new IllegalArgumentException("Segment count must be greater than zero");
        }
        if(!params.isTank) {
            throw new IllegalArgumentException("Params.isTank must be set to true");
        }

        this.specs = specs;
        this.params = params;

        _construct(specs.getMaxVelocity(), specs.getMaxAcceleration(), specs.getBaseWidth(), params.isTank, params.waypoints, 
                params.alpha, params.segmentCount, params.pathType.getJNIID());
    }
    public JNITankDriveTrajectory(RobotSpecs specs, JNITrajectoryParams params, long ptr) {
        this.specs = specs;
        this.params = params;
        _nativePtr = ptr;
    }

    @Override
    protected native void _destroy();

    @Override
    protected native int _getMomentCount();
    @Override
    protected native void _getMoments();
    protected TankDriveMoment[] momentsCache;
    @Override
    public TankDriveMoment[] getMoments() {
        if(momentsCache == null) {
            momentsCache = new TankDriveMoment[_getMomentCount()];
            _getMoments();
        }
        return momentsCache;
    }
    @Override
    public void clearMomentsCache() {
        momentsCache = null;
    }

    @Override
    protected native TankDriveMoment _get(double t);
    @Override
    public TankDriveMoment get(double t) {
        return (TankDriveMoment) super.get(t);
    }

    @Override
    protected native long _getPath();

    @Override
    public native double totalTime();

    private native long _mirrorLeftRight();
    @Override
    public JNITankDriveTrajectory mirrorLeftRight() {
        return new JNITankDriveTrajectory(specs, params, _mirrorLeftRight());
    }

    private native long _mirrorFrontBack();
    @Override
    public JNITankDriveTrajectory mirrorFrontBack() {
        return new JNITankDriveTrajectory(specs, params, _mirrorFrontBack());
    }

    private native long _retrace();
    @Override
    public JNITankDriveTrajectory retrace() {
        return new JNITankDriveTrajectory(specs, params, _retrace());
    }
}
