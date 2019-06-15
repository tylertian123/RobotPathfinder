package com.arctos6135.robotpathfinder.tests;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;

/**
 * This class is a helper used in random unit testing.
 * 
 * It generates random values used for testing and logs those values.
 */
public final class TestHelper {

    static {
        // Add a shutdown hook so that the logs are always written
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                try {
                    flushAll();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    // Maps classes to instances of TestHelper
    // Each class can only have one instance
    // Use ConcurrentHashMaps since tests might be multithreaded
    private static Map<Class<?>, TestHelper> instances = new ConcurrentHashMap<>();

    private TestHelper() {
    }

    /**
     * Gets the {@link TestHelper} instance for the class provided.
     * 
     * Each class has only one instance, which is shared by all the methods.
     * 
     * @param testClass The class of which to get the instance of
     * @return The {@link TestHelper} instance for that class
     */
    public static TestHelper getInstance(Class<?> testClass) {
        // If instance exists, return it
        if (instances.containsKey(testClass)) {
            return instances.get(testClass);
        }
        // Otherwise create the instance, add it to the map and return it
        else {
            TestHelper instance = new TestHelper();
            instances.put(testClass, instance);
            return instance;
        }
    }

    // Maps method/test names to logs
    // Each method/test has its own log
    private Map<String, StringBuffer> methodLogs = new ConcurrentHashMap<>();

    private StringBuffer getCallerLogs() {
        // Take the 4th element of the stack trace
        // 0 - getStackTrace()
        // 1 - getCallerLogs()
        // 2 - method calling getCallerLogs()
        // 3 - method calling the calling method
        String caller = Thread.currentThread().getStackTrace()[3].getMethodName();
        if (methodLogs.containsKey(caller)) {
            return methodLogs.get(caller);
        } else {
            StringBuffer log = new StringBuffer(caller + ":\n");
            methodLogs.put(caller, log);
            return log;
        }
    }

    private Random rand = new Random();

    /**
     * Generates a random {@code double} used for testing. Equivalent to
     * {@code getDouble(name, 0, max)}.
     * 
     * @param name The name of this {@code double}
     * @param max  The maximum value
     * @return A randomly generated value
     */
    public double getDouble(String name, double max) {
        // Generate the double value
        double val = rand.nextDouble() * max;
        // Log it
        StringBuffer log = getCallerLogs();
        log.append("[VALUE.DOUBLE] " + name + ": " + val + "\n");
        return val;
    }

    /**
     * Generates a random {@code double} used for testing. This method generates a
     * {@code double} that can take on any value between {@code min} and
     * {@code max}, inclusive.
     * 
     * @param name The name of this {@code double}
     * @param max  The maximum value
     * @param min  The minimum value
     * @return A randomly generated value.
     */
    public double getDouble(String name, double min, double max) {
        // Generate the double value
        double val = rand.nextDouble() * (max - min) + min;
        // Log it
        StringBuffer log = getCallerLogs();
        log.append("[VALUE.DOUBLE] " + name + ": " + val + "\n");
        return val;
    }

    /**
     * Generates a random {@code int} used for testing. Equivalent to
     * {@code getInt(name, 0, max)}.
     * 
     * @param name The name of this {@code int}
     * @param max  The maximum value
     * @return A randomly generated value
     */
    public int getInt(String name, int max) {
        // Generate the int value
        int val = rand.nextInt(max);
        // Log it
        StringBuffer log = getCallerLogs();
        log.append("[VALUE.INT] " + name + ": " + val + "\n");
        return val;
    }

    /**
     * Generates a random {@code int} used for testing. This method generates an
     * {@code int} that can take on any value between {@code min} (inclusive) and
     * {@code max} (exclusive).
     * 
     * @param name The name of this {@code int}
     * @param max  The maximum value
     * @param min  The minimum value
     * @return A randomly generated value.
     */
    public int getInt(String name, int min, int max) {
        // Generate the int value
        int val = rand.nextInt(max - min) + min;
        // Log it
        StringBuffer log = getCallerLogs();
        log.append("[VALUE.INT] " + name + ": " + val + "\n");
        return val;
    }

    /**
     * Logs a message.
     * 
     * @param message The message to log
     */
    public void logMessage(String message) {
        StringBuffer log = getCallerLogs();
        log.append("[MESSAGE] " + message + "\n");
    }

    /**
     * The folder where log files are written to.
     */
    public static final String LOG_LOCATION = "build" + File.separator + "testLogs" + File.separator;

    /**
     * Writes all logs into their corresponding files.
     * 
     * Files will have the name format {@code package/of/class/ClassName.log} and be
     * stored in {@link #LOG_LOCATION}.
     * 
     * This method is automatically called via a shutdown hook when the VM exits.
     * 
     * @throws IOException
     */
    public static void flushAll() throws IOException {
        for (var entry : instances.entrySet()) {
            // Get the log file name and path from the class name
            // Names are in the form package/of/class/Class.log
            String logFileName = entry.getKey().getName().replace(".", File.separator) + ".log";
            // Get the full path
            File logFile = new File(LOG_LOCATION + logFileName);
            // Create file and all dirs
            File parent = logFile.getParentFile();
            if (!parent.exists() && !parent.mkdirs()) {
                throw new IllegalStateException("Could not create directories: " + parent);
            }
            // Write logs
            FileWriter writer = new FileWriter(logFile, false);
            for (StringBuffer methodLog : entry.getValue().methodLogs.values()) {
                writer.append(methodLog.toString());
                // Separate using a blank line
                writer.append("\n");
            }
            writer.close();
        }
    }
}
