package robot.pathfinder.core;

import java.io.File;

public final class GlobalLibraryLoader {
    
    private GlobalLibraryLoader() {}

    private static boolean loaded = false;
    public static void load() {
        if(loaded) {
            return;
        }

        try {
            System.loadLibrary("RobotPathfinder");
            loaded = true;
        }
        catch(UnsatisfiedLinkError ule) {
            System.err.println("Warning: RobotPathfinder dynamic library not found in library path. Searching working directory...");
            try {
                System.load(System.getProperty("user.dir") + File.separator + System.mapLibraryName("RobotPathfinder"));
                loaded = true;
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
