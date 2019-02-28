package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.GlobalLibraryLoader;
import robot.pathfinder.core.JNIWaypoint;
import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.path.Path;

public class JNIBasicTrajectory implements Trajectory {

    static {
        GlobalLibraryLoader.load();
    }

    private long _nativePtr;

    private native void _construct(double maxV, double maxA, double baseWidth, boolean isTank, JNIWaypoint[] waypoints, 
            double alpha, int segmentCount, int type);

    public JNIBasicTrajectory(RobotSpecs specs, TrajectoryParams params) {
        
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
