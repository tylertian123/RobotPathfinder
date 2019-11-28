package com.arctos6135.robotpathfinder.core.lifecycle;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;

/**
 * This abstract class represents an object that has a native counterpart. It is
 * the superclass of all RobotPathfinder Paths and Trajectories.
 * <h2>Memory Management</h2>
 * <p>
 * Each JNIObject has a Java part (the object itself) and a part that resides in
 * native code (stored as a pointer casted into a {@code long}). Because these
 * objects contain handles to native resources that cannot be automatically
 * released by the JVM, the {@link #free()} or {@link #close()} method must be
 * called to free the native resource when the object is no longer needed.
 * </p>
 * <p>
 * Note: Almost all RobotPathfinder JNI classes have some kind of reference
 * counting. However, this reference count is only increased when an object is
 * created or copied by a method, and not when the reference is copied through
 * assignment. For example:
 * </p>
 * 
 * <pre>
 * Path p0 = someTrajectory.getPath();
 * Path p1 = someTrajectory.getPath();
 * p0.free();
 * p1.at(0); // This is valid, because the native resource was never freed due to
 *           // reference counting
 * </pre>
 * 
 * But:
 * 
 * <pre>
 * Path p0 = someTrajectory.getPath();
 * Path p1 = p0;
 * p0.free();
 * p1.at(0); // This will throw an IllegalStateException, since the native resource has
 *           // already been freed
 * </pre>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public abstract class JNIObject implements AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    /**
     * The address of the native resource.
     */
    protected long _nativePtr;

    /**
     * The {@link JNIObjectReference} associated with this {@link JNIObject}. Used
     * to deregister from the {@link GlobalLifeCycleManager}.
     */
    protected JNIObjectReference reference;

    /**
     * This should be implemented as a native method that frees the resources
     * associated with this object.
     */
    protected abstract void _destroy();

    /**
     * Frees the native resources aquired by this object. Calling this method
     * multiple times should have no effect, but the specifics depend on the
     * implementation.
     */
    public void free() {
        // Deregister from the GlobalLifeCycleManager as there's no longer a point
        // This also makes sure that when the same address is reused no strange errors
        // occur
        GlobalLifeCycleManager.deregister(this);
        _destroy();
    }

    /**
     * Frees the native resources aquired by this object. Calling this method
     * multiple times should have no effect, but the specifics depend on the
     * implementation.
     */
    @Override
    public void close() {
        // Deregister from the GlobalLifeCycleManager as there's no longer a point
        // This also makes sure that when the same address is reused no strange errors
        // occur
        GlobalLifeCycleManager.deregister(this);
        _destroy();
    }
}
