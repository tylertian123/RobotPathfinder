package com.arctos6135.robotpathfinder.core;

import java.io.File;

public final class GlobalLibraryLoader {
    
    private GlobalLibraryLoader() {}

    private static boolean loaded = false;
    public static void load() {
        if(loaded) {
            return;
        }

        try {
            System.out.println("Trying to find dynamic library on Java library path...");
            System.loadLibrary("RobotPathfinder");
            loaded = true;
            System.out.println("Library loaded successfully.");
        }
        catch(UnsatisfiedLinkError ule) {
            System.err.println("Warning: RobotPathfinder dynamic library not found in library path. Searching working directory...");
            System.out.println("Attempting to load " + System.getProperty("user.dir") + File.separator + System.mapLibraryName("RobotPathfinder"));
            try {
                System.load(System.getProperty("user.dir") + File.separator + System.mapLibraryName("RobotPathfinder"));
                loaded = true;
                System.out.println("Library loaded successfully.");
            }
            catch(UnsatisfiedLinkError ule2) {
                System.err.println("Critical error: Library cannot be loaded.");
            }
        }
    }
    public static boolean libraryLoaded() {
        return loaded;
    }
}
