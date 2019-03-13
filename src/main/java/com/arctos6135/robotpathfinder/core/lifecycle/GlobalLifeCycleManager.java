package com.arctos6135.robotpathfinder.core.lifecycle;

import java.lang.ref.ReferenceQueue;
import java.util.LinkedList;
import java.util.List;

/**
 * The {@code GlobalLifeCycleManager} is a singleton that manages the lifecycle of all {@link JNIObject}s 
 * in RobotPathfinder using {@code PhantomReference}s. This ensures that when an object is being garbage-collected
 * by the GC, its native counterpart is also freed and no memory leak happens.
 * 
 * However, this does not mean that {@link JNIObject#free()} is optional. The GC does not know how much memory each
 * native object takes up, and thus it will prioritize other large Java objects over {@link JNIObject}s due to its
 * small size in the JVM, which leads to inefficient memory management.
 * 
 * <p>
 * <b><em>This class is intended for internal use only. Use at your own risk.</em></b>
 * </p>
 */
public final class GlobalLifeCycleManager {
    
    private GlobalLifeCycleManager() {}

    protected static ReferenceQueue<JNIObject> referenceQueue = new ReferenceQueue<JNIObject>();
    protected static List<JNIObjectReference> references = new LinkedList<>();
    protected static ResourceDisposalThread resourceDisposalThread;
    protected static boolean initialized = false;

    protected static class ResourceDisposalThread extends Thread {
        
        public ResourceDisposalThread() {
            setDaemon(true);
        }

        @Override
        public void run() {
            while(true) {
                try {
                    JNIObjectReference ref = (JNIObjectReference) GlobalLifeCycleManager.referenceQueue.remove();
                    ref.freeResources();
                    references.remove(ref);
                    ref.clear();
                }
                catch(InterruptedException e) {
                }
            }
        }
    }

    /**
     * Initializes the {@link GlobalLifeCycleManager}. This will start a daemon thread dedicated to resource
     * disposal.
     */
    public static void initialize() {
        resourceDisposalThread = new ResourceDisposalThread();
        resourceDisposalThread.start();
        initialized = true;
    }
    
    /**
     * Gets whether or not the {@link GlobalLifeCycleManager} has been initialized.
     * 
     * @return Whether the {@link GlobalLifeCycleManager} has been initialized
     */
    public static boolean isInitialized() {
        return initialized;
    }

    /**
     * Registers an object to be managed by the {@link GlobalLifeCycleManager}.
     * 
     * @param obj The object to be managed
     */
    public static void register(JNIObject obj) {
        JNIObjectReference ref = new JNIObjectReference(obj, referenceQueue);
        references.add(ref);
    }
}

