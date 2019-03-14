package com.arctos6135.robotpathfinder.core.lifecycle;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;

/**
 * This abstract class represents an object that has a native counterpart. 
 * It is the superclass of all RobotPathfinder Paths and Trajectories.
 * <p>
 * Each JNIObject has a Java part (the object itself) and a part that resides in native code 
 * (stored as a pointer casted into a {@code long}). Because these objects contain handles
 * to native resources that cannot be automatically released by the JVM, the {@link #free()}
 * or {@link #close()} method must be called to free the native resource when the object is
 * no longer needed.
 * </p>
 * <p><em>
 * Note: Almost all RobotPathfinder JNI classes have some kind of reference counting. 
 * However, this reference count is only increased when an object is created or copied 
 * by a method, and not when the reference is copied through assignment. <br>
 * For example:
 * <pre>{@code
 * Path p0 = someTrajectory.getPath();
 * Path p1 = someTrajectory.getPath();
 * p0.free();
 * p1.at(0); // This is valid, because the native resource was never freed due to reference counting
 * }</pre>
 * But:
 * <pre>{@code
 * Path p0 = someTrajectory.getPath();
 * Path p1 = p0;
 * p0.free();
 * p1.at(0); // This will throw an IllegalStateException, since the native resource has already been freed
 * }</pre>
 * </em></p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public abstract class JNIObject implements AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    protected long _nativePtr;

    /**
     * This should be implemented as a native method that frees the resources associated with this object.
     */
    protected abstract void _destroy();
    
    /**
     * Frees the native resources aquired by this object. 
     * Calling this method multiple times should have no effect, but the specifics depend on the implementation.
     */
    public void free() {
        _destroy();
    }
    /**
     * Frees the native resources aquired by this object. 
     * Calling this method multiple times should have no effect, but the specifics depend on the implementation.
     */
    @Override
    public void close() {
        _destroy();
    }
}
