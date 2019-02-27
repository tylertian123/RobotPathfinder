package robot.pathfinder.core.path;

import robot.pathfinder.core.JNIWaypoint;
import robot.pathfinder.core.Waypoint;

public class JNIPath {
    static {
        System.loadLibrary("RobotPathfinder");
    }

    private long _nativePtr;

    private static final int PT_BEZIER = 1;
    private static final int PT_CUBIC_HERMITE = 2;
    private static final int PT_QUINTIC_HERMITE = 3;
    private native void construct(JNIWaypoint[] waypoints, double alpha, int type);
}
