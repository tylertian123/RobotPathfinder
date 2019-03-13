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

public class BasicTest {
    
    @Test
    public void testPathAllMethods() {
        Waypoint[] waypoints = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(10, 10, 0),
            new Waypoint(12.34, 5.67, Math.PI),
        };
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

    @Test
    public void testTankDriveTrajectoryAllMethods() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
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

    @Test
    public void testBasicTrajectoryAllMethods() {
        RobotSpecs specs = new RobotSpecs(5, 3, 1);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(10, 10, Math.PI / 2),
            new Waypoint(0, 20, Math.PI),
        };
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

    @Test
    public void testPathIllegalStateException() {
        Waypoint[] waypoints = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(10, 10, 0),
            new Waypoint(12.34, 5.67, Math.PI),
        };
        Path path = new Path(waypoints, 30, PathType.BEZIER);
        Path p = path;
        path.close();

        boolean exception = false;
        try {
            p.at(0);
        }
        catch(IllegalStateException e) {
            exception = true;
        }
        assertThat(exception, is(true));
    }
}
