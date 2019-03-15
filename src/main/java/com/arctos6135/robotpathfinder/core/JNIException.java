package com.arctos6135.robotpathfinder.core;

/**
 * Indicates that something has gone wrong with JNI. It is currently unused.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class JNIException extends RuntimeException {

    private static final long serialVersionUID = 7424682501003868216L;

    public JNIException() {
        super();
    }

    public JNIException(String msg) {
        super(msg);
    }
}
