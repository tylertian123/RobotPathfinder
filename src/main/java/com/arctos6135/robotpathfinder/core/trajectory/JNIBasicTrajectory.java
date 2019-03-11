package com.arctos6135.robotpathfinder.core.trajectory;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;
import com.arctos6135.robotpathfinder.core.JNITrajectoryParams;
import com.arctos6135.robotpathfinder.core.JNIWaypoint;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.path.Path;

public class JNIBasicTrajectory implements JNITrajectory, AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    private long _nativePtr;

    private native void _construct(double maxV, double maxA, double baseWidth, boolean isTank, JNIWaypoint[] waypoints, 
            double alpha, int segmentCount, int type);

    public JNIBasicTrajectory(RobotSpecs specs, JNITrajectoryParams params) {
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

        _construct(specs.getMaxVelocity(), specs.getMaxAcceleration(), specs.getBaseWidth(), params.isTank, params.waypoints, 
                params.alpha, params.segmentCount, params.pathType.getJNIID());
    }

    private native void _destroy();
    public void free() {
        _destroy();
    }
    @Override
    public void finalize() {
        _destroy();
    }
    @Override
    public void close() {
        _destroy();
    }

    @Override
    public Moment[] getMoments() {
        return null;
    }

    @Override
    public Moment get(double t) {
        return null;
    }

    @Override
    public Path getPath() {
        return null;
    }

    @Override
    public double totalTime() {
        return 0;
    }

    @Override
    public RobotSpecs getRobotSpecs() {
        return null;
    }

    @Override
    public JNITrajectoryParams getGenerationParams() {
        return null;
    }

    @Override
    public JNITrajectory mirrorLeftRight() {
        return null;
    }

    @Override
    public JNITrajectory mirrorFrontBack() {
        return null;
    }

    @Override
    public JNITrajectory retrace() {
        return null;
    }

}
