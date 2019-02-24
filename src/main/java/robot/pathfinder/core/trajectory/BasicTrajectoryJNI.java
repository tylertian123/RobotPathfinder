package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.Path;

public class BasicTrajectoryJNI implements Trajectory {
    static {
        System.loadLibrary("RobotPathfinder");
    }

    // Pointer to the native object
    private long _nativePtr;

    private native void construct(double maxVelocity, double maxAcceleration, double baseWidth,
            Waypoint[] waypoints, double alpha, int segmentCount, boolean isTank, int pathType);

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
    public TrajectoryParams getGenerationParams() {
        return null;
    }

    @Override
    public Trajectory mirrorLeftRight() {
        return null;
    }

    @Override
    public Trajectory mirrorFrontBack() {
        return null;
    }

    @Override
    public Trajectory retrace() {
        return null;
    }

}
