package robot.pathfinder.core.path;

import java.io.File;

import robot.pathfinder.core.JNIWaypoint;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;

public class JNIPath implements AutoCloseable {
    static {
        try {
            System.loadLibrary("RobotPathfinder");
        }
        catch(UnsatisfiedLinkError ule) {
            System.err.println("Warning: RobotPathfinder dynamic library not found in library path. Searching working directory...");
            try {
                System.loadLibrary(System.getProperty("user.dir") + File.separator + System.mapLibraryName("RobotPathfinder"));
            }
            catch(UnsatisfiedLinkError ule2) {
                System.err.println("Critical error: Library cannot be loaded.");
            }
        }
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

    private native void _setBaseRadius(double radius);
    private native void _setBackwards(boolean backwards);
    protected double radius;
    protected boolean backwards = false;

    public void setBaseRadius(double radius) {
        _setBaseRadius(radius);
    }
    public void setDrivingBackwards(boolean backwards) {
        _setBackwards(backwards);
    }
    public double getBaseRadius() {
        return radius;
    }
    public boolean getDrivingBackwards() {
        return backwards;
    }
}
