package com.arctos6135.robotpathfinder.core.lifecycle;

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;

public abstract class JNIObject implements AutoCloseable {

    static {
        GlobalLibraryLoader.load();
    }

    protected long _nativePtr;

    // Native method
    protected abstract void _destroy();
    
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
}
