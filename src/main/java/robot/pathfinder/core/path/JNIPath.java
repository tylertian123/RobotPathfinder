package robot.pathfinder.core.path;

import robot.pathfinder.core.GlobalLibraryLoader;
import robot.pathfinder.core.JNIWaypoint;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.util.Pair;

public class JNIPath implements AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    private long _nativePtr;

    private static final int PT_BEZIER = 1;
    private static final int PT_CUBIC_HERMITE = 2;
    private static final int PT_QUINTIC_HERMITE = 3;
    private native void _construct(JNIWaypoint[] waypoints, double alpha, int type);

    protected PathType type;
    protected Waypoint[] waypoints;
    protected double alpha;
    public JNIPath(Waypoint[] waypoints, double alpha, PathType type) {
        this.type = type;
        this.waypoints = waypoints;
        this.alpha = alpha;
        if(waypoints.length < 2) {
            throw new IllegalArgumentException("Not enough waypoints");
        }

        JNIWaypoint[] jniWaypoints = new JNIWaypoint[waypoints.length];
        for(int i = 0; i < waypoints.length; i ++) {
            if(waypoints[i] instanceof WaypointEx) {
                jniWaypoints[i] = new JNIWaypoint(waypoints[i].getX(), waypoints[i].getY(), waypoints[i].getHeading(), ((WaypointEx) waypoints[i]).getVelocity());
            }
            else {
                jniWaypoints[i] = new JNIWaypoint(waypoints[i].getX(), waypoints[i].getY(), waypoints[i].getHeading());
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
    private JNIPath(Waypoint[] waypoints, double alpha, PathType type, long ptr) {
        this.waypoints = waypoints;
        this.alpha = alpha;
        this.type = type;
        _nativePtr = ptr;
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
        this.radius = radius;
        _setBaseRadius(radius);
    }
    public void setDrivingBackwards(boolean backwards) {
        this.backwards = backwards;
        _setBackwards(backwards);
    }
    public double getBaseRadius() {
        return radius;
    }
    public boolean getDrivingBackwards() {
        return backwards;
    }
    
    public Waypoint[] getWaypoints() {
        return waypoints;
    }
    public double getAlpha() {
        return alpha;
    }

    public native Vec2D at(double time);
    public native Vec2D derivAt(double time);
    public native Vec2D secondDerivAt(double time);
    public native Pair<Vec2D, Vec2D> wheelsAt(double time);

    private native double _computeLen(int points);
    private native double _s2T(double s);
    private native double _t2S(double t);

    protected double length = Double.NaN;
    public double computeLen(int points) {
        length = _computeLen(points);
        return length;
    }
    public double getLength() {
        if(length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return length;
    }
    public double s2T(double s) {
        if(length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return _s2T(s);
    }
    public double t2S(double t) {
        if(length == Double.NaN) {
            throw new IllegalStateException("Length has not been computed");
        }
        return _t2S(t);
    }
    
    private native long _mirrorLeftRight();
    private native long _mirrorFrontBack();
    private native long _retrace();

    public JNIPath mirrorLeftRight() {
        return new JNIPath(waypoints, alpha, type, _mirrorLeftRight());
    }
    public JNIPath mirrorFrontBack() {
        return new JNIPath(waypoints, alpha, type, _mirrorFrontBack());
    }
    public JNIPath retrace() {
        return new JNIPath(waypoints, alpha, type, _retrace());
    }
}
