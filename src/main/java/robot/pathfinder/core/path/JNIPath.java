package robot.pathfinder.core.path;

import robot.pathfinder.core.JNIWaypoint;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;

public class JNIPath {
    static {
        System.loadLibrary("RobotPathfinder");
    }

    private long _nativePtr;

    private static final int PT_BEZIER = 1;
    private static final int PT_CUBIC_HERMITE = 2;
    private static final int PT_QUINTIC_HERMITE = 3;
    private native void _construct(JNIWaypoint[] waypoints, double alpha, int type);

    protected PathType type;
    public JNIPath(Waypoint[] waypoints, double alpha, PathType type) {
        this.type = type;
        if(waypoints.length < 2) {
            throw new IllegalArgumentException("Not enough waypoints");
        }

        JNIWaypoint[] jniWaypoints = new JNIWaypoint[waypoints.length];
        for(int i = 0; i < waypoints.length; i ++) {
            if(waypoints[i] instanceof WaypointEx) {
                jniWaypoints[i] = new JNIWaypoint(waypoints[i].getX(), waypoints[i].getY(), waypoints[i].getHeading());
            }
            else {
                jniWaypoints[i] = new JNIWaypoint(waypoints[i].getX(), waypoints[i].getY(), waypoints[i].getHeading(), ((WaypointEx) waypoints[i]).getVelocity());
            }
        }
        int iType;
        switch(type) {
        case BEZIER:
            iType = PT_BEZIER;
            break;
        case CUBIC_HERMITE:
            iType = PT_CUBIC_HERMITE;
            break;
        case QUINTIC_HERMITE:
            iType = PT_QUINTIC_HERMITE;
            break;
        default: 
            iType = 0;
            break;
        }
        
        _construct(jniWaypoints, alpha, iType);
    }
}
