package com.arctos6135.robotpathfinder.tests;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;

/**
 * This class is a helper used in random unit testing.
 * 
 * It generates random values used for testing and logs those values.
 * 
 * @author Tyler Tian
 * @since 3.0.0
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

                String TRAVIS = System.getenv("TRAVIS");
                if (TRAVIS != null && TRAVIS.equals("true")) {
                    printAll();
                }
            }
        });
    }

    // Maps classes to instances of TestHelper
    // Each class can only have one instance
    // Use ConcurrentHashMaps since tests might be multithreaded
    private static Map<Class<?>, TestHelper> instances = new ConcurrentHashMap<>();

    // The file where logs will be written to
    private File logFile;

    private TestHelper(Class<?> testClass) {
        // Names are in the form package/of/class/Class.log
        logFile = new File(LOG_LOCATION + testClass.getName().replace(".", File.separator) + ".log");

        // Add itself to the instance map
        instances.put(testClass, this);
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
        // Otherwise create the instance
        else {
            // The constructor adds itself to the instance map
            return new TestHelper(testClass);
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

    // Gets the method name of the caller of the method calling this method.
    private String getCallerName() {
        // Take the 4th element of the stack trace
        // 0 - getStackTrace()
        // 1 - getCallerName()
        // 2 - method calling getCallerName()
        // 3 - method calling the calling method
        return Thread.currentThread().getStackTrace()[3].getMethodName();
    }

    private Random rand = new Random();

    private boolean replicateLastTest = false;

    /**
     * Retrieves whether this {@link TestHelper} instance is configured to replicate
     * the last test.
     * 
     * When a {@link TestHelper} instance is configured to replicate the last test,
     * it reads a log file that it has written previously and extracts all the
     * generated values from that file. All subsequent calls to
     * {@link #getDouble(String, double, double)}, {@link #getInt(String, int, int)}
     * and their variants will now return the logged value instead of a randomly
     * generated one. This essentially replicates the behavior of the logged test,
     * hence the name.
     * 
     * @return Whether to replicate the last test
     */
    public boolean getReplicateLastTest() {
        return replicateLastTest;
    }

    /**
     * Sets whether this {@link TestHelper} instance is configured to replicate the
     * last test. The name of the log file to read is automatically inferred from
     * the class name.
     * 
     * When a {@link TestHelper} instance is configured to replicate the last test,
     * it reads a log file that it has written previously and extracts all the
     * generated values from that file. All subsequent calls to
     * {@link #getDouble(String, double, double)}, {@link #getInt(String, int, int)}
     * and their variants will now return the logged value instead of a randomly
     * generated one. This essentially replicates the behavior of the logged test,
     * hence the name.
     * 
     * @param replicateLastTest Whether to replicate the last test
     * @throws IOException           If something goes wrong when reading the log
     *                               file
     * @throws IllegalStateException If the log file does not exist or is invalid
     */
    public void setReplicateLastTest(boolean replicateLastTest) throws IOException {
        this.replicateLastTest = replicateLastTest;
        loadLoggedValues(logFile);
    }

    /**
     * Sets whether this {@link TestHelper} instance is configured to replicate the
     * last test.
     * 
     * When a {@link TestHelper} instance is configured to replicate the last test,
     * it reads a log file that it has written previously and extracts all the
     * generated values from that file. All subsequent calls to
     * {@link #getDouble(String, double, double)}, {@link #getInt(String, int, int)}
     * and their variants will now return the logged value instead of a randomly
     * generated one. This essentially replicates the behavior of the logged test,
     * hence the name.
     * 
     * @param replicateLastTest Whether to replicate the last test
     * @param logFile           The log file to read and extract values from
     * @throws IOException           If something goes wrong when reading the log
     *                               file
     * @throws IllegalStateException If the log file does not exist or is invalid
     */
    public void setReplicateLastTest(boolean replicateLastTest, File logFile) throws IOException {
        this.replicateLastTest = replicateLastTest;
        loadLoggedValues(logFile);
    }

    /**
     * Generates a random {@code double} used for testing. Equivalent to
     * {@code getDouble(name, 0, max)}.
     * 
     * If configured to replicate the last test, this method will return the same
     * value as written in the log file.
     * 
     * @param name The name of this {@code double}
     * @param max  The maximum value
     * @return A randomly generated value
     * @throws IllegalArgumentException If configured to replicate the last test,
     *                                  but the log file does not contain an entry
     *                                  for the calling method or name
     * @see #setReplicateLastTest(boolean, File)
     */
    public double getDouble(String name, double max) {
        if (replicateLastTest) {
            String callerName = getCallerName();
            if (!loggedValues.containsKey(callerName)) {
                throw new IllegalArgumentException("Loaded log file does not contain entry for method: " + callerName);
            }
            if (!loggedValues.get(callerName).doubles.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.get(callerName).doubles.get(name);
        }

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
     * If configured to replicate the last test, this method will return the same
     * value as written in the log file.
     * 
     * @param name The name of this {@code double}
     * @param max  The maximum value
     * @param min  The minimum value
     * @return A randomly generated value.
     * @throws IllegalArgumentException If configured to replicate the last test,
     *                                  but the log file does not contain an entry
     *                                  for the calling method or name
     * @see #setReplicateLastTest(boolean, File)
     */
    public double getDouble(String name, double min, double max) {
        if (replicateLastTest) {
            String callerName = getCallerName();
            if (!loggedValues.containsKey(callerName)) {
                throw new IllegalArgumentException("Loaded log file does not contain entry for method: " + callerName);
            }
            if (!loggedValues.get(callerName).doubles.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.get(callerName).doubles.get(name);
        }

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
     * If configured to replicate the last test, this method will return the same
     * value as written in the log file.
     * 
     * @param name The name of this {@code int}
     * @param max  The maximum value
     * @return A randomly generated value
     * @throws IllegalArgumentException If configured to replicate the last test,
     *                                  but the log file does not contain an entry
     *                                  for the calling method or name
     * @see #setReplicateLastTest(boolean, File)
     */
    public int getInt(String name, int max) {
        if (replicateLastTest) {
            String callerName = getCallerName();
            if (!loggedValues.containsKey(callerName)) {
                throw new IllegalArgumentException("Loaded log file does not contain entry for method: " + callerName);
            }
            if (!loggedValues.get(callerName).ints.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.get(callerName).ints.get(name);
        }

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
     * If configured to replicate the last test, this method will return the same
     * value as written in the log file.
     * 
     * @param name The name of this {@code int}
     * @param max  The maximum value
     * @param min  The minimum value
     * @return A randomly generated value.
     * @throws IllegalArgumentException If configured to replicate the last test,
     *                                  but the log file does not contain an entry
     *                                  for the calling method or name
     * @see #setReplicateLastTest(boolean, File)
     */
    public int getInt(String name, int min, int max) {
        if (replicateLastTest) {
            String callerName = getCallerName();
            if (!loggedValues.containsKey(callerName)) {
                throw new IllegalArgumentException("Loaded log file does not contain entry for method: " + callerName);
            }
            if (!loggedValues.get(callerName).ints.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.get(callerName).ints.get(name);
        }

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
     * @throws IOException If something goes wrong when writing the logs
     */
    public static void flushAll() throws IOException {
        for (var entry : instances.entrySet()) {
            // Don't overwrite the log file if replicateLastTest is true
            if (entry.getValue().replicateLastTest) {
                continue;
            }
            File logFile = entry.getValue().logFile;
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

    /**
     * Prints all logs to stdout.
     * 
     * This method is automatically called via a VM shutdown hook if the $TRAVIS
     * environment variable is set to "true".
     */
    public static void printAll() {
        System.out.println("********** TEST LOGS **********");
        for (var entry : instances.entrySet()) {
            File logFile = entry.getValue().logFile;

            System.out.println("(FILE: " + logFile.getPath() + ")");
            for (StringBuffer methodLog : entry.getValue().methodLogs.values()) {
                System.out.println(methodLog.toString());
            }
        }
    }

    /**
     * Clears all logs. Warning: Logs will be lost forever if they were not written!
     */
    public static void clearAll() {
        for (TestHelper instance : instances.values()) {
            instance.methodLogs = new ConcurrentHashMap<>();
        }
    }

    // A class that represents all values generated for a method/test
    // The members map names to values (doubles and ints)
    private static class GeneratedValues {

        public Map<String, Double> doubles = new ConcurrentHashMap<>();
        public Map<String, Integer> ints = new ConcurrentHashMap<>();
    }

    // This maps method names to their generated values
    // Usually null, unless loadLoggedValues() is called
    private Map<String, GeneratedValues> loggedValues;

    /**
     * Loads all logged values for all methods of the associated class.
     */
    private void loadLoggedValues(File logFile) throws IOException {
        if (!logFile.exists()) {
            throw new IllegalStateException("TestHelper log not found: " + logFile);
        }

        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(logFile));
        } catch (FileNotFoundException e) {
            // This should never happen!
            e.printStackTrace();
            return;
        }

        loggedValues = new ConcurrentHashMap<>();
        GeneratedValues values = null;
        // Read line by line
        String line;
        while ((line = reader.readLine()) != null) {
            // Skip empty lines
            if (line.length() == 0) {
                continue;
            }
            // If line does not start with a label and ends with a colon, it's the start of
            // a new method
            if (line.charAt(0) != '[' && line.charAt(line.length() - 1) == ':') {
                // Get the method name
                String methodName = line.substring(0, line.length() - 1);
                // Create a new values object and add it to the map
                values = new GeneratedValues();
                loggedValues.put(methodName, values);
            } else if (line.charAt(0) == '[') {
                try {
                    // If there was no method name, throw exception
                    if (values == null) {
                        reader.close();
                        throw new IllegalStateException("Invalid log file: " + logFile);
                    }
                    // Find where the label ends
                    int labelEnd = line.indexOf(']');
                    String label = line.substring(1, labelEnd);
                    // If it's not a value, skip
                    if (!label.startsWith("VALUE")) {
                        continue;
                    }

                    // Find the substring containing the name and value
                    // Skip the label end ] and space after it
                    String[] s = line.substring(labelEnd + 2).split(":");
                    String name = s[0];
                    // Chop off the first character from value since it's a space
                    String valueStr = s[1].substring(1);

                    // Get value type from label
                    switch (label) {
                    case "VALUE.DOUBLE": {
                        double value = Double.parseDouble(valueStr);
                        values.doubles.put(name, value);
                        break;
                    }
                    case "VALUE.INT": {
                        int value = Integer.parseInt(valueStr);
                        values.ints.put(name, value);
                        break;
                    }
                    default:
                        break;
                    }
                }
                // These exceptions mean that the log file is not valid
                catch (StringIndexOutOfBoundsException | NumberFormatException e) {
                    reader.close();
                    throw new IllegalStateException("Invalid log file: " + logFile);
                }
            } else {
                reader.close();
                throw new IllegalStateException("Invalid log file: " + logFile);
            }
        }

        reader.close();
    }
}
