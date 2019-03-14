package com.arctos6135.robotpathfinder.core;

import java.io.File;

/**
 * The {@code GlobalLibraryLoader} is a static class whose only purpose is to
 * load the RobotPathfinder shared native library.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public final class GlobalLibraryLoader {

    private GlobalLibraryLoader() {
    }

    private static boolean loaded = false;

    /**
     * Attempts to load the RobotPathfinder shared native library. If the library is
     * already loaded, this method will have no effect.
     * <p>
     * This method will first attempt to load the library by directly invoking
     * {@code System.loadLibrary()}. If the library is not in the Java library path,
     * an error message will be printed, and this method will attempt to load the
     * library from the current directory ("user.dir"). If the library cannot be
     * found, an error message will be printed, and calls to
     * {@link #libraryLoaded()} will return {@code false}.
     * </p>
     */
    public static void load() {
        if (loaded) {
            return;
        }

        try {
            System.out.println("Trying to find dynamic library on Java library path...");
            System.loadLibrary("RobotPathfinder");
            loaded = true;
            System.out.println("Library loaded successfully.");
        } catch (UnsatisfiedLinkError ule) {
            System.err.println(
                    "Warning: RobotPathfinder dynamic library not found in library path. Searching working directory...");
            System.out.println("Attempting to load " + System.getProperty("user.dir") + File.separator
                    + System.mapLibraryName("RobotPathfinder"));
            try {
                System.load(System.getProperty("user.dir") + File.separator + System.mapLibraryName("RobotPathfinder"));
                loaded = true;
                System.out.println("Library loaded successfully.");
            } catch (UnsatisfiedLinkError ule2) {
                System.err.println("Critical error: Library cannot be loaded.");
            }
        }
    }

    /**
     * Retrieves whether the RobotPathfinder shared library has been loaded.
     * <p>
     * This method will return {@code false} if the {@link #load()} method was never
     * called, or if the loading failed.
     * </p>
     */
    public static boolean libraryLoaded() {
        return loaded;
    }
}
