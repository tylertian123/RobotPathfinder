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
 */
public abstract class JNIObject implements AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    protected long _nativePtr;

    // Native method
    protected abstract void _destroy();
    
    /**
     * Frees the native resources aquired by this object. 
     * Calling this method multiple times will have no effect.
     * <p>
     * </p>
     */
    public void free() {
        _destroy();
    }
    /**
     * Frees the native resources aquired by this object. 
     * Calling this method multiple times will have no effect.
     * <p>
     * </p>
     */
    @Override
    public void close() {
        _destroy();
    }
}
