package com.arctos6135.robotpathfinder.tests;

import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.rules.TestName;

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
                // If the TRAVIS environment variable is set, then we're on Travis CI
                // Print out all the logs since we can't retrieve the files from a CI build
                String TRAVIS = System.getenv("TRAVIS");
                if (TRAVIS != null && TRAVIS.equals("true")) {
                    printAll();
                } else {
                    // Otherwise try to flush all log buffers
                    try {
                        flushAll();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
    }

    // A list of all instances
    // This is used to flush all buffers
    private static final Collection<TestHelper> instances = new ConcurrentLinkedQueue<>();

    // The file where logs will be written to
    private final File logFile;

    // Used to generate values for getInt, getDouble, etc
    private final Random rand = new Random();

    // Replicates the last test if set to true
    private boolean replicateLastTest = false;

    // Default for replicateLastTest
    private static boolean defaultReplicateLastTest = false;

    // The logs for this test
    private StringBuilder log = new StringBuilder();

    public TestHelper(Class<?> testClass, TestName testName) {
        this(testClass, testName.getMethodName());
    }

    public TestHelper(Class<?> testClass, String testName) {
        replicateLastTest = defaultReplicateLastTest;

        // Names are in the form package/of/class/ClassClassName/testName.log
        logFile = new File(
                LOG_LOCATION + testClass.getName().replace(".", File.separator) + File.separator + testName + ".log");

        if (replicateLastTest) {
            try {
                loadLoggedValues(logFile);
            } catch (IOException e) {
                throw new IllegalArgumentException(
                        "Default replicate last test was set, but loading of this test's logs failed!", e);
            }
        }

        // Add itself to the instance list
        instances.add(this);
    }

    public static boolean getDefaultReplicateLastTest() {
        return defaultReplicateLastTest;
    }

    public static void setDefaultReplicateLastTest(boolean replicate) {
        defaultReplicateLastTest = replicate;
    }

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
        return getDouble(name, 0, max);
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
            if (!loggedValues.doubles.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.doubles.get(name);
        }

        // Generate the double value
        double val = rand.nextDouble() * (max - min) + min;
        // Log it
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
        return getInt(name, 0, max);
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
            if (!loggedValues.ints.containsKey(name)) {
                throw new IllegalArgumentException(
                        "Loaded log file does not contain entry for value with name: " + name);
            }
            return loggedValues.ints.get(name);
        }

        // Generate the int value
        int val = rand.nextInt(max - min) + min;
        // Log it
        log.append("[VALUE.INT] " + name + ": " + val + "\n");
        return val;
    }

    /**
     * Logs a message.
     * 
     * @param message The message to log
     */
    public void logMessage(String message) {
        log.append("[MESSAGE] " + message + "\n");
    }

    public void write() throws IOException {
        // Don't overwrite the log file if replicateLastTest is true
        if (replicateLastTest) {
            return;
        }
        // Create file and all dirs
        File parent = logFile.getParentFile();
        if (!parent.exists() && !parent.mkdirs()) {
            throw new IOException("Could not create directories: " + parent);
        }
        // Write logs
        // Overwrite original
        FileWriter writer = new FileWriter(logFile, false);
        writer.write(log.toString());
        writer.close();
    }

    /**
     * The folder where log files are written to.
     */
    public static final String LOG_LOCATION = "build" + File.separator + "testLogs" + File.separator;

    /**
     * Writes all logs into their corresponding files.
     * 
     * Files will have the name format
     * {@code package/of/class/ClassName/testName.log} and be stored in
     * {@link #LOG_LOCATION}.
     * 
     * This method is automatically called via a shutdown hook when the VM exits.
     * 
     * @throws IOException If something goes wrong when writing the logs
     */
    public static void flushAll() throws IOException {
        for (TestHelper instance : instances) {
            instance.write();
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
        for (TestHelper instance : instances) {
            File logFile = instance.logFile;

            System.out.println("(FILE: " + logFile.getPath() + ")");
            System.out.println(instance.log.toString());
        }
    }

    /**
     * Clears all logs. Warning: Logs will be lost forever if they were not written!
     */
    public static void clearAll() {
        for (TestHelper instance : instances) {
            instance.log = new StringBuilder();
        }
    }

    // A class that represents all values generated for a method/test
    private static class GeneratedValues {

        public Map<String, Double> doubles = new ConcurrentHashMap<>();
        public Map<String, Integer> ints = new ConcurrentHashMap<>();
    }

    // This holds all generated values
    // Usually null, unless loadLoggedValues() is called
    private GeneratedValues loggedValues;

    /**
     * Loads all logged values for all methods of the associated class.
     */
    private void loadLoggedValues(File logFile) throws IOException {
        if (!logFile.exists()) {
            throw new IllegalArgumentException("TestHelper log not found: " + logFile);
        }

        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(logFile));
        } catch (FileNotFoundException e) {
            // This should never happen!
            e.printStackTrace();
            return;
        }

        loggedValues = new GeneratedValues();
        // Read line by line
        String line;
        while ((line = reader.readLine()) != null) {
            // Skip empty lines
            if (line.length() == 0) {
                continue;
            }
            
            try {
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
                    loggedValues.doubles.put(name, value);
                    break;
                }
                case "VALUE.INT": {
                    int value = Integer.parseInt(valueStr);
                    loggedValues.ints.put(name, value);
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
        }

        reader.close();
    }

    private static boolean overridesEquals(Class<?> clazz) {
        if (clazz.isPrimitive()) {
            return true;
        }
        Method equals = null;
        try {
            equals = clazz.getMethod("equals", Object.class);
        } catch (NoSuchMethodException e) {
            // This should never happen
            e.printStackTrace();
            return false;
        }

        return equals.getDeclaringClass().equals(clazz);
    }

    private static final class ObjectPairReference {

        Object refA;
        Object refB;

        public ObjectPairReference(Object referenceA, Object referenceB) {
            refA = referenceA;
            refB = referenceB;
        }

        @Override
        public boolean equals(Object other) {
            if (other == this) {
                return true;
            }
            if (!(other instanceof ObjectPairReference)) {
                return false;
            }
            ObjectPairReference o = (ObjectPairReference) other;
            return refA == o.refA && refB == o.refB;
        }

        @Override
        public int hashCode() {
            return Objects.hash(System.identityHashCode(refA), System.identityHashCode(refB));
        }
    }

    /**
     * Asserts that all fields in two objects are equal.
     * <p>
     * This method uses reflection to find out all fields of an object, and compares
     * each of the fields, regardless of visibility. If the field's type does not
     * override {@link Object#equals(Object)}, then this method will recurse on that
     * field to compare it. If the values of any field aren't equal in the two
     * objects, the {@link org.junit.Assert#fail() fail()} method from JUnit will be
     * called.
     * </p>
     * <p>
     * If the objects given override {@link Object#equals(Object)}, it will be used
     * instead.
     * </p>
     * <p>
     * This is equivalent to calling
     * {@link #assertAllFieldsEqual(Object, Object, int)} with a max recursion depth
     * of 255.
     * </p>
     * 
     * @param a Any object
     * @param b An object to compare it with
     */
    public static void assertAllFieldsEqual(Object a, Object b) {
        assertAllFieldsEqual(a, b, 255);
    }

    /**
     * Asserts that all fields in two objects are equal.
     * <p>
     * This method uses reflection to find out all fields of an object, and compares
     * each of the fields, regardless of visibility. If the field's type does not
     * override {@link Object#equals(Object)}, then this method will recurse on that
     * field to compare it. If the values of any field aren't equal in the two
     * objects, the {@link org.junit.Assert#fail() fail()} method from JUnit will be
     * called.
     * </p>
     * <p>
     * If the objects given override {@link Object#equals(Object)}, it will be used
     * instead.
     * </p>
     * 
     * @param a                 Any object
     * @param b                 An object to compare it with
     * @param maxRecursionDepth The maximum allowed recursion depth
     */
    public static void assertAllFieldsEqual(Object a, Object b, int maxRecursionDepth) {
        assertAllFieldsEqual(a, b, maxRecursionDepth, 0, new HashSet<>());
    }

    private static void assertAllFieldsEqual(Object a, Object b, int maxRecursionDepth, int recursionDepth,
            Set<ObjectPairReference> visited) {
        // Test for reference equality first as the simple case
        // Also handles nulls
        if (a == b) {
            return;
        }

        // Test to see if this exact pair of values has already been visited
        ObjectPairReference ref = new ObjectPairReference(a, b);
        if (visited.contains(ref)) {
            // Skip if already visited to avoid infinite loops
            return;
        }
        // Otherwise add the pair to the set
        visited.add(ref);

        Class<?> clazz = a.getClass();
        // Make sure the two classes are equal
        if (!clazz.equals(b.getClass())) {
            fail("The two objects being compared have different classes (" + clazz.getName() + " vs "
                    + b.getClass().getName() + ")!");
        }
        // First check if equals() was already overridden
        if (overridesEquals(clazz)) {
            // Directly compare with equals()
            // This can avoid illegal reflection operations on native stuff like strings and
            // arrays
            if (a.equals(b)) {
                return;
            } else {
                fail("The two objects being compared are not equal!");
            }
        }

        // Next, check for arrays
        if (clazz.isArray()) {
            // Check if it's a primitive array
            Class<?> componentType = clazz.getComponentType();
            if (componentType.isPrimitive()) {
                // Use the Array class to deal with primitive arrays
                int len = Array.getLength(a);
                if (len != Array.getLength(b)) {
                    fail("The length of the two arrays aren't equal (" + len + " vs " + Array.getLength(b) + ")!");
                }
                for (int i = 0; i < len; i++) {
                    Object aElem = Array.get(a, i);
                    Object bElem = Array.get(b, i);
                    // Directly use equals() for comparison
                    if (!aElem.equals(bElem)) {
                        fail("Elements at index " + i + " aren't equal (" + aElem + " vs " + bElem + ")!");
                    }
                }
            } else {
                // Directly cast to object array
                Object[] aArr = (Object[]) a;
                Object[] bArr = (Object[]) b;

                if (aArr.length != bArr.length) {
                    fail("The length of the two arrays aren't equal (" + aArr.length + " vs " + bArr.length + ")!");
                }
                for (int i = 0; i < aArr.length; i++) {
                    try {
                        // Recurse
                        assertAllFieldsEqual(aArr[i], bArr[i], maxRecursionDepth, recursionDepth + 1, visited);
                    }
                    // Catch the exception thrown by possible test failures
                    // Report more relevant info instead
                    catch (Throwable t) {
                        fail("Elements at index " + i + " aren't equal (" + aArr[i] + " vs " + bArr[i] + ")!");
                    }
                }
            }
            return;
        }
        // Get all declared fields and compare them
        Field[] fields = clazz.getDeclaredFields();
        for (Field field : fields) {
            // This field is inserted by jacoco
            // skip it
            if (field.getName().equals("$jacocoData")) {
                continue;
            }
            field.setAccessible(true);
            try {
                Object aValue = field.get(a);
                Object bValue = field.get(b);
                // Test for reference equality first to handle nulls and simple cases
                if (aValue != bValue) {
                    // Now see if Object.equals() was overridden
                    Class<?> fieldClass = field.getType();
                    // Test to see if equals() was indeed implemented
                    if (overridesEquals(fieldClass)) {
                        // If it is, then use it
                        if (!aValue.equals(bValue)) {
                            fail("The field '" + field.getName() + "' is not equal: " + aValue + " vs " + bValue);
                        }
                    } else {
                        // If equals() was not implemented, then recurse if allowed
                        if (recursionDepth >= maxRecursionDepth) {
                            fail("Max recursion depth for field comparison reached on class: " + fieldClass.getName());
                        }

                        try {
                            // Recurse
                            assertAllFieldsEqual(aValue, bValue, maxRecursionDepth, recursionDepth + 1, visited);
                        }
                        // Catch the exception thrown by possible test failures
                        // Report more relevant field names instead
                        catch (Throwable t) {
                            fail("The field '" + field.getName() + "' is not equal: " + aValue + " vs " + bValue);
                        }
                    }
                }
            } catch (IllegalAccessException e) {
                // Should never happen
                fail("IllegalAccessException: " + e.getMessage());
            }
        }
    }
}
