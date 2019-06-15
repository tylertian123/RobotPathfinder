package com.arctos6135.robotpathfinder.tests;

import static org.hamcrest.core.Is.is;
import static org.hamcrest.core.IsNot.not;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThat;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;

import org.junit.Test;

/**
 * This class contains miscellaneous basic tests for some classes.
 */
public class BasicTest {

    /**
     * Tests all methods of {@link Path} to ensure that the JNI works properly.
     * 
     * This executes the methods:
     * <ul>
     * <li>{@link Path#Path(Waypoint[], double, PathType)}</li>
     * <li>{@link Path#computeLen(int)}</li>
     * <li>{@link Path#at(double)}</li>
     * <li>{@link Path#derivAt(double)}</li>
     * <li>{@link Path#secondDerivAt(double)}</li>
     * <li>{@link Path#setBaseRadius(double)}</li>
     * <li>{@link Path#setDrivingBackwards(boolean)}</li>
     * <li>{@link Path#t2S(double)}</li>
     * <li>{@link Path#s2T(double)}</li>
     * <li>{@link Path#_updateWaypoints()}</li>
     * <li>{@link Path#mirrorLeftRight()}</li>
     * <li>{@link Path#mirrorFrontBack()}</li>
     * <li>{@link Path#retrace()}</li>
     * <li>{@link Path#close()}</li>
     * </ul>
     */
    @Test
    public void testPathAllMethods() {
        Waypoint[] waypoints = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(10, 10, 0),
                new Waypoint(12.34, 5.67, Math.PI), };
        Path bezierPath = new Path(waypoints, 30, PathType.BEZIER);
        Path cubicPath = new Path(waypoints, 30, PathType.CUBIC_HERMITE);
        Path quinticPath = new Path(waypoints, 30, PathType.QUINTIC_HERMITE);

        assertThat(bezierPath.computeLen(100), is(not(Double.NaN)));
        assertNotNull(bezierPath.at(0));
        assertNotNull(bezierPath.derivAt(0));
        assertNotNull(bezierPath.secondDerivAt(0));
        bezierPath.setBaseRadius(1);
        bezierPath.setDrivingBackwards(true);
        assertThat(bezierPath.t2S(0), is(not(Double.NaN)));
        assertThat(bezierPath.s2T(0), is(not(Double.NaN)));
        bezierPath._updateWaypoints();
        assertNotNull(bezierPath.mirrorLeftRight());
        assertNotNull(bezierPath.mirrorFrontBack());
        assertNotNull(bezierPath.retrace());

        bezierPath.close();
        cubicPath.close();
        quinticPath.close();
    }

    /**
     * Tests all methods of {@link TankDriveTrajectory} to ensure that the JNI works
     * properly.
     * 
     * This executes the methods:
     * <ul>
     * <li>{@link TankDriveTrajectory#TankDriveTrajectory(RobotSpecs, TrajectoryParams)}</li>
     * <li>{@link TankDriveTrajectory#getMoments()}</li>
     * <li>{@link TankDriveTrajectory#get(double)}</li>
     * <li>{@link TankDriveTrajectory#getPath()}</li>
     * <li>{@link TankDriveTrajectory#mirrorLeftRight()}</li>
     * <li>{@link TankDriveTrajectory#mirrorFrontBack()}</li>
     * <li>{@link TankDriveTrajectory#retrace()}</li>
     * <li>{@link TankDriveTrajectory#close()}</li>
     * </ul>
     */
    @Test
    public void testTankDriveTrajectoryAllMethods() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
        assertNotNull(traj.getMoments());
        assertNotNull(traj.get(0));
        assertNotNull(traj.getPath());
        assertNotNull(traj.mirrorFrontBack());
        assertNotNull(traj.mirrorLeftRight());
        assertNotNull(traj.retrace());
        traj.close();
    }

    /**
     * Tests all methods of {@link BasicTrajectory} to ensure that the JNI works
     * properly.
     * 
     * This executes the methods:
     * <ul>
     * <li>{@link BasicTrajectory#BasicTrajectory(RobotSpecs, TrajectoryParams)}</li>
     * <li>{@link BasicTrajectory#getMoments()}</li>
     * <li>{@link BasicTrajectory#get(double)}</li>
     * <li>{@link BasicTrajectory#getPath()}</li>
     * <li>{@link BasicTrajectory#mirrorLeftRight()}</li>
     * <li>{@link BasicTrajectory#mirrorFrontBack()}</li>
     * <li>{@link BasicTrajectory#retrace()}</li>
     * <li>{@link BasicTrajectory#close()}</li>
     * </ul>
     */
    @Test
    public void testBasicTrajectoryAllMethods() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        BasicTrajectory traj = new BasicTrajectory(specs, params);
        assertNotNull(traj.getMoments());
        assertNotNull(traj.get(0));
        assertNotNull(traj.getPath());
        assertNotNull(traj.mirrorFrontBack());
        assertNotNull(traj.mirrorLeftRight());
        assertNotNull(traj.retrace());
        traj.close();
    }

    /**
     * Tests that {@code Path} methods throw an {@link IllegalStateException} if the
     * object has already been freed.
     */
    @Test
    public void testPathIllegalStateException() {
        Waypoint[] waypoints = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(10, 10, 0),
                new Waypoint(12.34, 5.67, Math.PI), };
        Path path = new Path(waypoints, 30, PathType.BEZIER);
        Path p = path;
        path.close();

        boolean exception = false;
        try {
            p.at(0);
        } catch (IllegalStateException e) {
            exception = true;
        }
        assertThat(exception, is(true));
    }

    /**
     * Tests that {@code BasicTrajectory} methods throw an
     * {@link IllegalStateException} if the object has already been freed.
     */
    @Test
    public void testBasicTrajectoryIllegalStateException() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        BasicTrajectory traj = new BasicTrajectory(specs, params);
        BasicTrajectory t = traj;
        traj.close();

        boolean exception = false;
        try {
            t.get(0);
        } catch (IllegalStateException e) {
            exception = true;
        }
        assertThat(exception, is(true));
    }

    /**
     * Tests that {@code TankDriveTrajectory} methods throw an
     * {@link IllegalStateException} if the object has already been freed.
     */
    @Test
    public void testTankDriveTrajectoryIllegalStateException() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] { new Waypoint(0, 0, Math.PI / 2), new Waypoint(10, 10, Math.PI / 2),
                new Waypoint(0, 20, Math.PI), };
        params.alpha = 20;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.sampleCount = 1000;

        TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
        TankDriveTrajectory t = traj;
        traj.close();

        boolean exception = false;
        try {
            t.get(0);
        } catch (IllegalStateException e) {
            exception = true;
        }
        assertThat(exception, is(true));
    }
}
