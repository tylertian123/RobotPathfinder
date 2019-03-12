package com.arctos6135.robotpathfinder.core.lifecycle;

import java.lang.ref.ReferenceQueue;
import java.util.LinkedList;
import java.util.List;

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

    public static void initialize() {
        resourceDisposalThread = new ResourceDisposalThread();
        resourceDisposalThread.start();
        initialized = true;
    }
    
    public static boolean isInitialized() {
        return initialized;
    }

    public static void register(JNIObject obj) {
        JNIObjectReference ref = new JNIObjectReference(obj, referenceQueue);
        references.add(ref);
    }
}

