package com.arctos6135.robotpathfinder.core.lifecycle;

import java.lang.ref.PhantomReference;
import java.lang.ref.ReferenceQueue;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;

public class JNIObjectReference extends PhantomReference<JNIObject> {

    static {
        GlobalLibraryLoader.load();
    }

    protected long objNativePtr;

    public JNIObjectReference(JNIObject obj, ReferenceQueue<? super JNIObject> refQueue) {
        super(obj, refQueue);
        objNativePtr = obj._nativePtr;
    }

    private static native void _freeObject(long ptr);
    
    public void freeResources() {
        _freeObject(objNativePtr);
    }
}

