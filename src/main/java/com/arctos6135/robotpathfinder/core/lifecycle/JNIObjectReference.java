package com.arctos6135.robotpathfinder.core.lifecycle;

import java.lang.ref.PhantomReference;
import java.lang.ref.ReferenceQueue;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;

/**
 * The {@code JNIObjectReference} is a {@code PhantomReference<JNIObject>} that also holds data
 * about a {@link JNIObject}'s native pointer and provides a way to free the object after it 
 * has been finalized. It is used by the {@link GlobalLifeCycleManager}.
 * 
 * <p>
 * <b><em>This class is intended for internal use only. Use at your own risk.</em></b>
 * </p>
 */
public class JNIObjectReference extends PhantomReference<JNIObject> {

    static {
        GlobalLibraryLoader.load();
    }

    protected long objNativePtr;

    /**
     * Creates a new {@link JNIObjectReference} of the specified object with the specified 
     * reference queue.
     * 
     * @param obj The object to be referred
     * @param refQueue A {@code ReferenceQueue} that the reference will be placed in
     */
    public JNIObjectReference(JNIObject obj, ReferenceQueue<? super JNIObject> refQueue) {
        super(obj, refQueue);
        objNativePtr = obj._nativePtr;
    }

    /**
     * This native method will free the native resource pointed to by {@code ptr}, if 
     * it has not already been freed.
     * 
     * @param ptr The pointer to free
     */
    private static native void _freeObject(long ptr);
    /**
     * Frees the native resources associated with the object being referenced, if it
     * has not already been freed.
     */
    public void freeResources() {
        _freeObject(objNativePtr);
    }
}

