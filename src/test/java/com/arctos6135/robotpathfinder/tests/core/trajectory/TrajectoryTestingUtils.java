package com.arctos6135.robotpathfinder.tests.core.trajectory;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.tests.TestHelper;

/**
 * This class contains a collection of handy methods for generating objects used
 * in unit testing for trajectories and more.
 * 
 * @author Tyler Tian
 */
public final class TrajectoryTestingUtils {

    private TrajectoryTestingUtils() {
    }

    // The upper bound of typical random values
    private static final double RANDOM_MAX = 100000;

    /**
     * Generates a random {@link RobotSpecs} object.
     * 
     * @param helper            A {@link TestHelper} instance to get random numbers
     *                          from
     * @param generateBaseWidth Whether or not to generate and set the base width of
     *                          the object
     * @return A randomly generated {@link RobotSpecs} object
     */
    public static RobotSpecs getRandomRobotSpecs(TestHelper helper, boolean generateBaseWidth) {
        double maxV = helper.getDouble("robotSpecsMaxV", RANDOM_MAX);
        double maxA = helper.getDouble("robotSpecsMaxA", RANDOM_MAX);

        if (generateBaseWidth) {
            double baseWidth = helper.getDouble("robotSpecsBaseWidth", RANDOM_MAX);
            return new RobotSpecs(maxV, maxA, baseWidth);
        } else {
            return new RobotSpecs(maxV, maxA);
        }
    }

    /**
     * Generates an array of random {@link Waypoint}s.
     * <p>
     * The array will contain a random number of randomly generated instances.
     * </p>
     * 
     * @param helper A {@link TestHelper} instance to get random numbers from
     * @return An array of randomly generated {@link Waypoint}s
     */
    public static Waypoint[] getRandomWaypoints(TestHelper helper) {
        return getRandomWaypoints(helper, -1);
    }

    /**
     * Generates an array of random {@link Waypoint}s.
     * 
     * @param helper A {@link TestHelper} instance to get random numbers from
     * @param count  The number of waypoints to generate; if zero or negative, a
     *               random number of waypoints will be generated
     * @return An array of randomly generated {@link Waypoint}s
     */
    public static Waypoint[] getRandomWaypoints(TestHelper helper, int count) {
        if (count <= 0) {
            // Keep this number relatively small for speed
            count = helper.getInt("waypointsCount", 2, 100);
        }

        Waypoint[] waypoints = new Waypoint[count];
        for (int i = 0; i < count; i++) {
            waypoints[i] = new Waypoint(helper.getDouble("waypoint" + i + "X", -RANDOM_MAX, RANDOM_MAX),
                    helper.getDouble("waypoint" + i + "Y", -RANDOM_MAX, RANDOM_MAX),
                    helper.getDouble("waypoint" + i + "Heading", -Math.PI, Math.PI));
        }
        return waypoints;
    }

    /**
     * Generates a random {@link TrajectoryParams} object with random waypoints.
     * 
     * @param helper A {@link TestHelper} instance to get random numbers from
     * @return A randomly generated {@link TrajectoryParams} object
     */
    public static TrajectoryParams getRandomTrajectoryParams(TestHelper helper) {
        return getRandomTrajectoryParams(helper, getRandomWaypoints(helper));
    }

    /**
     * Generates a random {@link TrajectoryParams} object.
     * 
     * @param helper    A {@link TestHelper} instance to get random numbers from
     * @param waypoints The waypoints to use
     * @return A randomly generated {@link TrajectoryParams} object
     */
    public static TrajectoryParams getRandomTrajectoryParams(TestHelper helper, Waypoint[] waypoints) {
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = waypoints;
        params.alpha = helper.getDouble("trajectoryParamsAlpha", RANDOM_MAX);
        params.sampleCount = helper.getInt("trajectoryParamsSampleCount", 3, 1000);
        params.pathType = getRandomPathType(helper);

        return params;
    }

    /**
     * Generates a random {@link PathType}.
     * 
     * @param helper A {@link TestHelper} instance to get random numbers from
     * @return A random {@link PathType}
     */
    public static PathType getRandomPathType(TestHelper helper) {
        int type = helper.getInt("pathType", 2);
        switch (type) {
        case 0:
            return PathType.BEZIER;
        case 1:
            return PathType.CUBIC_HERMITE;
        case 2:
        default:
            return PathType.QUINTIC_HERMITE;
        }
    }
}
